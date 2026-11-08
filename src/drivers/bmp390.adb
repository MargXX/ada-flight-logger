with HAL;     use HAL;
with HAL.I2C; use HAL.I2C;
with RP.Device;

package body BMP390 is

   --  8-bit address form with SDO high, because rp2040_hal shifts right
   Addr : constant I2C_Address := 16#EE#;

   ERR_REG   : constant UInt8 := 16#02#;
   STATUS    : constant UInt8 := 16#03#;
   DATA_0    : constant UInt8 := 16#04#;
   PWR_CTRL  : constant UInt8 := 16#1B#;
   OSR       : constant UInt8 := 16#1C#;
   ODR       : constant UInt8 := 16#1D#;
   CONFIG    : constant UInt8 := 16#1F#;
   NVM_START : constant UInt8 := 16#31#;
   CMD       : constant UInt8 := 16#7E#;

   Press_And_Temp_On : constant UInt8 := 16#03#;
   Conf_Err_Bit      : constant UInt8 := 16#04#;
   Soft_Reset        : constant UInt8 := 16#B6#;

   --  STATUS.drdy_press and STATUS.drdy_temp, cleared by reading DATA
   Data_Ready_Bits   : constant UInt8 := 16#60#;

   --  the slowest allowed rate is 12.5 Hz, so a new result is due within
   --  80 ms in normal mode. Wait a little longer than that.
   Data_Wait_Ms : constant := 100;

   --  short so a stuck bus can't stall the logging loop for long
   Timeout_Ms : constant := 5;

   --  Write one register. Ok is False if the transaction fails.
   procedure Write (Reg : UInt8; Value : UInt8; Ok : out Boolean) is
      Status : I2C_Status;
   begin
      RP.Device.I2CM_0.Mem_Write
        (Addr          => Addr,
         Mem_Addr      => UInt16 (Reg),
         Mem_Addr_Size => Memory_Size_8b,
         Data          => I2C_Data'(1 => Value),
         Status        => Status,
         Timeout       => Timeout_Ms);
      Ok := Status = HAL.I2C.Ok;
   end Write;

   --  Read registers starting at Reg. Ok is False if the transaction fails.
   procedure Read (Reg : UInt8; Data : out I2C_Data; Ok : out Boolean) is
      Status : I2C_Status;
   begin
      RP.Device.I2CM_0.Mem_Read
        (Addr          => Addr,
         Mem_Addr      => UInt16 (Reg),
         Mem_Addr_Size => Memory_Size_8b,
         Data          => Data,
         Status        => Status,
         Timeout       => Timeout_Ms);
      Ok := Status = HAL.I2C.Ok;
   end Read;

   --  Apply the settings and start measuring. Stops at the first failure.
   procedure Configure
     (Mode        : Power_Mode;
      Pressure    : Oversampling;
      Temperature : Oversampling;
      Rate        : Output_Rate;
      Ok          : out Boolean)
   is
      --  osr_p is bits 2:0 and osr_t is bits 5:3
      OSR_Value : constant UInt8 :=
        UInt8 (Oversampling'Enum_Rep (Pressure))
        + UInt8 (Oversampling'Enum_Rep (Temperature)) * 8;

      --  mode is bits 5:4
      Mode_Bits : constant UInt8 := UInt8 (Power_Mode'Enum_Rep (Mode)) * 16;

      Error   : I2C_Data (1 .. 1);
      Power   : I2C_Data (1 .. 1);
      Flags   : I2C_Data (1 .. 1);
      Discard : I2C_Data (1 .. 6);
   begin
      --  Reset to power-on defaults, so the chip is asleep and idle however
      --  it was left. It keeps power across a Pico reset, and a mode change
      --  sent mid-measurement can be ignored, which froze the readings once.
      Write (CMD, Soft_Reset, Ok);

      --  the datasheet allows 2 ms for the reset
      RP.Device.Timer.Delay_Milliseconds (10);

      if Ok then
         Write (OSR, OSR_Value, Ok);
      end if;
      if Ok then
         Write (ODR, UInt8 (Output_Rate'Enum_Rep (Rate)), Ok);
      end if;

      --  IIR filter off
      if Ok then
         Write (CONFIG, 0, Ok);
      end if;

      --  enable both measurements and start the chosen mode
      if Ok then
         Write (PWR_CTRL, Press_And_Temp_On + Mode_Bits, Ok);
      end if;

      --  check that the chip accepted this combination
      if Ok then
         Read (ERR_REG, Error, Ok);
      end if;
      if Ok then
         Ok := (Error (1) and Conf_Err_Bit) = 0;
      end if;

      --  check that the mode and enables actually took
      if Ok then
         Read (PWR_CTRL, Power, Ok);
      end if;
      if Ok then
         Ok := (Power (1) and 16#33#) = Press_And_Temp_On + Mode_Bits;
      end if;

      --  in normal mode, wait for a fresh result to prove it is measuring
      if Ok and then Mode = Normal then
         --  reading the data clears the ready flags
         Read (DATA_0, Discard, Ok);

         if Ok then
            Ok := False;
            for I in 1 .. Data_Wait_Ms loop
               RP.Device.Timer.Delay_Milliseconds (1);
               declare
                  Read_Ok : Boolean;
               begin
                  Read (STATUS, Flags, Read_Ok);
                  if Read_Ok
                    and then (Flags (1) and Data_Ready_Bits) = Data_Ready_Bits
                  then
                     Ok := True;
                     exit;
                  end if;
               end;
            end loop;
         end if;
      end if;
   end Configure;

   --  Read the latest raw pressure and temperature.
   procedure Read_Raw
     (Pressure    : out Unsigned_32;
      Temperature : out Unsigned_32;
      Ok          : out Boolean)
   is
      Data : I2C_Data (1 .. 6);

      --  join 3 bytes, which arrive low byte first
      function U24 (First : Positive) return Unsigned_32 is
        (Unsigned_32 (Data (First))
         + Unsigned_32 (Data (First + 1)) * 256
         + Unsigned_32 (Data (First + 2)) * 65_536);
   begin
      Pressure := 0;
      Temperature := 0;

      --  pressure and temperature sit back to back, so read them together
      Read (DATA_0, Data, Ok);

      if Ok then
         Pressure := U24 (1);
         Temperature := U24 (4);
      end if;
   end Read_Raw;

   --  Read the factory calibration block.
   procedure Read_Calibration
     (Data : out Byte_Array;
      Ok   : out Boolean)
   is
      Raw : I2C_Data (1 .. Calibration_Size);
   begin
      Data := (others => 0);
      Read (NVM_START, Raw, Ok);

      --  copy across, since HAL bytes and Byte_Array are different types
      if Ok then
         for I in Raw'Range loop
            Data (Data'First + I - Raw'First) := Byte (Raw (I));
         end loop;
      end if;
   end Read_Calibration;

end BMP390;
