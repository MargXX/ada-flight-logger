--  flight_reader.adb
--  Sends each written flash page over UART as one line of 512 hex characters,
--  with nothing else on the line. Stops at the first blank page. Read-only.
--  The page layout is in sample_log.ads.
--
--  LED: blinks for 5 seconds so a capture can be started, toggles while
--  reading, and goes solid when done. Fast blink means no flash was found.

with Byte_Types; use Byte_Types;

with Debug_UART;
with Pico;
with RP.Clock;
with RP.Device;
with RP.GPIO;
with Sample_Log;
with W25Q128;

procedure Flight_Reader is
   Page : Sample_Log.Page_Buffer;

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
      Fault_Blink;
   end loop;

   --  give time to start capturing on the computer
   for I in 1 .. 10 loop
      Pico.LED.Toggle;
      RP.Device.Timer.Delay_Milliseconds (500);
   end loop;

   --  send pages in order until one was never written
   for Index in 0 .. W25Q128.Page_Count - 1 loop
      W25Q128.Read (Index * W25Q128.Page_Size, Page);
      exit when Sample_Log.Is_Blank
        (Page (Sample_Log.Sequence_Offset .. Page'Last));
      Debug_UART.Put_Hex_Line (Byte_Array (Page));
      Pico.LED.Toggle;
   end loop;

   --  solid LED means finished
   Pico.LED.Set;
   loop
      null;
   end loop;
end Flight_Reader;
