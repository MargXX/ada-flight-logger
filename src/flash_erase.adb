--  flash_erase.adb
--  Erases the whole flash chip, after E is sent over UART to confirm.
--  Save any data with flight_reader first. Can take a few minutes.
--
--  LED: on while erasing and off when done. Fast blink means no flash found.

with Debug_UART; use Debug_UART;
with Pico;
with RP.Clock;
with RP.Device;
with RP.GPIO;
with W25Q128;

procedure Flash_Erase is
   Ok : Boolean;

   --  Blink the LED fast for about a second to show a fault. Blocks while
   --  blinking, so the caller can loop on it and retry.
   procedure Fault_Blink is
   begin
      for I in 1 .. 20 loop
         Pico.LED.Toggle;
         RP.Device.Timer.Delay_Milliseconds (50);
      end loop;
   end Fault_Blink;

begin
   --  start the clocks, timer and GPIO, and make the LED an output
   RP.Clock.Initialize (Pico.XOSC_Frequency);
   RP.Clock.Enable (RP.Clock.PERI);
   RP.Device.Timer.Enable;
   RP.GPIO.Enable;
   Pico.LED.Configure (RP.GPIO.Output);

   Debug_UART.Initialize;
   W25Q128.Initialize;

   --  fast blink and retry until the flash answers
   while not W25Q128.Is_Present loop
      Put_Line ("Flash not found");
      Fault_Blink;
   end loop;

   --  ask first, so a power cycle alone can never erase flight data
   Put_Line ("This erases ALL flash data. Send E to continue.");
   Wait_For ('E');

   Put_Line ("Erasing...");
   Pico.LED.Set;
   W25Q128.Erase (W25Q128.Whole_Chip, 0, Ok);
   Pico.LED.Clear;

   Put_Line (if Ok then "Erase done" else "Erase timed out");
   loop
      null;
   end loop;
end Flash_Erase;
