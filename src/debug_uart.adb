with HAL;        use HAL;
with HAL.UART;   use HAL.UART;
with Interfaces; use Interfaces;
with RP.Device;
with RP.GPIO;    use RP.GPIO;
with RP.UART;

package body Debug_UART is

   Port : RP.UART.UART_Port renames RP.Device.UART_1;
   TX   : GPIO_Point := (Pin => 8);
   RX   : GPIO_Point := (Pin => 9);

   Hex_Digits : constant String := "0123456789ABCDEF";

   --  Set up the pins and the UART.
   procedure Initialize is
   begin
      TX.Configure (Output, Pull_Up, RP.GPIO.UART);
      RX.Configure (Input, Floating, RP.GPIO.UART);
      Port.Configure
        (Config =>
           (Baud      => 115_200,
            Word_Size => 8,
            Parity    => False,
            Stop_Bits => 1,
            others    => <>));
   end Initialize;

   --  Send text. Transmit waits when the FIFO is full, so no delay is needed.
   procedure Put (Text : String) is
      Data   : UART_Data_8b (1 .. Text'Length);
      Status : UART_Status;
   begin
      --  copy the characters into the byte array the HAL wants
      for I in Text'Range loop
         Data (I - Text'First + 1) := Character'Pos (Text (I));
      end loop;
      Port.Transmit (Data, Status);
   end Put;

   --  Send text followed by CR LF.
   procedure Put_Line (Text : String) is
   begin
      Put (Text & ASCII.CR & ASCII.LF);
   end Put_Line;

   --  Send the bytes as one line of uppercase hex.
   procedure Put_Hex_Line (Data : Byte_Array) is
      Line : String (1 .. Data'Length * 2);
      Next : Positive := 1;
   begin
      for B of Data loop
         --  high nibble, then low nibble
         Line (Next) := Hex_Digits (Hex_Digits'First + Natural (B / 16));
         Line (Next + 1) := Hex_Digits (Hex_Digits'First + Natural (B mod 16));
         Next := Next + 2;
      end loop;
      Put_Line (Line);
   end Put_Hex_Line;

   --  Block until the given character arrives on RX.
   procedure Wait_For (C : Character) is
      Data   : UART_Data_8b (1 .. 1);
      Status : UART_Status;
   begin
      --  a timeout just means nothing was sent yet, so keep waiting
      loop
         Port.Receive (Data, Status, Timeout => 1_000);
         exit when Status = Ok and then Data (1) = Character'Pos (C);
      end loop;
   end Wait_For;

end Debug_UART;
