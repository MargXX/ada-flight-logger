with HAL.SPI;
with Interfaces; use Interfaces;
with RP.Device;
with RP.GPIO;    use RP.GPIO;
with RP.SPI;     use RP.SPI;
with RP.Timer;   use RP.Timer;
with RP2040_SVD.SPI;

package body W25Q128 is

   CMD_PAGE_PROGRAM : constant Byte := 16#02#;
   CMD_READ_DATA    : constant Byte := 16#03#;
   CMD_READ_STATUS  : constant Byte := 16#05#;
   CMD_WRITE_ENABLE : constant Byte := 16#06#;
   CMD_ERASE_4K     : constant Byte := 16#20#;
   CMD_ERASE_32K    : constant Byte := 16#52#;
   CMD_ERASE_CHIP   : constant Byte := 16#C7#;
   CMD_ERASE_64K    : constant Byte := 16#D8#;
   CMD_JEDEC_ID     : constant Byte := 16#9F#;

   --  set well above the datasheet maximums
   Program_Timeout_Ms : constant := 20;
   Block_Timeout_Ms   : constant := 5_000;
   Chip_Timeout_Ms    : constant := 300_000;

   MISO : GPIO_Point := (Pin => 16);
   CS   : GPIO_Point := (Pin => 17);
   SCK  : GPIO_Point := (Pin => 18);
   MOSI : GPIO_Point := (Pin => 19);

   SPI_Port : RP.SPI.SPI_Port renames RP.Device.SPI_0;

   --  Pull CS low to start a command.
   procedure Select_Chip is
   begin
      CS.Clear;
   end Select_Chip;

   --  Pull CS high to end a command.
   procedure Deselect_Chip is
   begin
      CS.Set;
      --  1 us comfortably covers the chip's minimum time between commands
      Busy_Wait_Until (Clock + 1);
   end Deselect_Chip;

   --  Clock one byte out and one byte in, and wait for both to finish.
   --  Uses the registers directly because the HAL's Receive does not generate
   --  clocks and its Transmit returns before bytes finish sending.
   procedure Exchange (Out_Byte : Byte; In_Byte : out Byte) is
      Periph : RP2040_SVD.SPI.SPI_Peripheral renames SPI_Port.Periph.all;
   begin
      --  wait for room in the TX FIFO, then send
      while not Periph.SSPSR.TNF loop
         null;
      end loop;
      Periph.SSPDR.DATA := RP2040_SVD.SPI.SSPDR_DATA_Field (Out_Byte);

      --  wait for the byte clocked in at the same time, then read it
      while not Periph.SSPSR.RNE loop
         null;
      end loop;
      In_Byte := Byte (Periph.SSPDR.DATA);
   end Exchange;

   --  Clock one byte out and throw away what comes back.
   procedure Send (Out_Byte : Byte) is
      Ignored : Byte;
   begin
      Exchange (Out_Byte, Ignored);
   end Send;

   --  Send a 3-byte address, highest byte first.
   procedure Send_Address (Address : Flash_Address) is
   begin
      Send (Byte (Address / 65_536));
      Send (Byte ((Address / 256) mod 256));
      Send (Byte (Address mod 256));
   end Send_Address;

   --  Unlock the chip for one program or erase. Needed before each one.
   procedure Write_Enable is
   begin
      Select_Chip;
      Send (CMD_WRITE_ENABLE);
      Deselect_Chip;
   end Write_Enable;

   --  Poll until the busy bit clears. Ok is False on timeout.
   procedure Wait_Until_Ready (Timeout_Ms : Natural; Ok : out Boolean) is
      Deadline : constant Time := Clock + Milliseconds (Timeout_Ms);
      Status   : Byte;
   begin
      Select_Chip;
      Send (CMD_READ_STATUS);

      --  the chip keeps sending status while CS stays low
      loop
         Exchange (0, Status);
         Ok := (Status and 16#01#) = 0;  --  bit 0 is the busy bit
         exit when Ok or else Clock >= Deadline;
      end loop;

      Deselect_Chip;
   end Wait_Until_Ready;

   --  Set up the pins and the SPI port.
   procedure Initialize is
   begin
      MISO.Configure (Input, Floating, RP.GPIO.SPI);
      SCK.Configure (Output, Floating, RP.GPIO.SPI);
      MOSI.Configure (Output, Floating, RP.GPIO.SPI);

      --  CS is a plain GPIO so it stays low across a whole command
      CS.Configure (Output, Floating);
      CS.Set;

      --  give the chip time to power up
      RP.Device.Timer.Delay_Milliseconds (10);

      SPI_Port.Configure
        (Config =>
           (Role      => RP.SPI.Master,
            Baud      => 10_000_000,
            Data_Size => HAL.SPI.Data_Size_8b,
            Polarity  => RP.SPI.Active_Low,   --  CPOL=0, clock idles low
            Phase     => RP.SPI.Rising_Edge,  --  CPHA=0
            others    => <>));
   end Initialize;

   --  Returns True if the chip answers with the expected JEDEC ID.
   function Is_Present return Boolean is
      ID : Byte_Array (1 .. 3);
   begin
      Select_Chip;
      Send (CMD_JEDEC_ID);
      for B of ID loop
         Exchange (0, B);
      end loop;
      Deselect_Chip;

      --  Winbond, SPI flash, 128 Mbit
      return ID = (16#EF#, 16#40#, 16#18#);
   end Is_Present;

   --  Fill Data with the bytes starting at Address.
   procedure Read (Address : Flash_Address; Data : out Byte_Array) is
   begin
      Select_Chip;
      Send (CMD_READ_DATA);
      Send_Address (Address);
      for B of Data loop
         Exchange (0, B);
      end loop;
      Deselect_Chip;
   end Read;

   --  Write one full page and wait for it to finish.
   procedure Program_Page
     (Address : Flash_Address;
      Data    : Byte_Array;
      Ok      : out Boolean) is
   begin
      Write_Enable;

      --  send the command, address and page data as one transaction
      Select_Chip;
      Send (CMD_PAGE_PROGRAM);
      Send_Address (Address);
      for B of Data loop
         Send (B);
      end loop;

      --  the chip starts writing when CS goes high
      Deselect_Chip;
      Wait_Until_Ready (Program_Timeout_Ms, Ok);
   end Program_Page;

   --  Erase the chosen size and wait for it to finish.
   procedure Erase
     (Size    : Erase_Size;
      Address : Flash_Address;
      Ok      : out Boolean) is
   begin
      Write_Enable;

      --  send the command for this size, with an address unless whole chip
      Select_Chip;
      case Size is
         when Sector_4_KB =>
            Send (CMD_ERASE_4K);
            Send_Address (Address);
         when Block_32_KB =>
            Send (CMD_ERASE_32K);
            Send_Address (Address);
         when Block_64_KB =>
            Send (CMD_ERASE_64K);
            Send_Address (Address);
         when Whole_Chip =>
            Send (CMD_ERASE_CHIP);
      end case;

      --  the chip starts erasing when CS goes high
      Deselect_Chip;
      Wait_Until_Ready
        ((if Size = Whole_Chip then Chip_Timeout_Ms else Block_Timeout_Ms),
         Ok);
   end Erase;

end W25Q128;
