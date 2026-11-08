--  flash_dump_swd.adb
--  Reads flash into RAM on request so a debugger can copy it out over SWD.
--  For when the UART is unreliable. Read-only.
--  The handshake is in swd_mailbox.ads.
--
--  LED: toggles once per request. Fast blink means no flash was found.

with Byte_Types; use Byte_Types;
with Interfaces; use Interfaces;

with Pico;
with RP.Clock;
with RP.Device;
with RP.GPIO;
with SWD_Mailbox;
with W25Q128;

procedure Flash_Dump_SWD is
   Page : Byte_Array (0 .. W25Q128.Page_Size - 1);

   --  Blink the LED fast for about a second to show a fault.
   procedure Fault_Blink is
   begin
      for I in 1 .. 20 loop
         Pico.LED.Toggle;
         RP.Device.Timer.Delay_Milliseconds (50);
      end loop;
   end Fault_Blink;

   --  Copy the requested pages into the mailbox buffer.
   --  Pages past the end of the chip are skipped.
   procedure Fill_Buffer is
      First : constant Natural := Natural (SWD_Mailbox.Start_Page);
      Count : constant Natural :=
        Natural'Min (Natural (SWD_Mailbox.Page_Count), SWD_Mailbox.Max_Pages);
   begin
      for I in 0 .. Count - 1 loop
         exit when First + I >= W25Q128.Page_Count;
         W25Q128.Read ((First + I) * W25Q128.Page_Size, Page);

         --  copy one page into its slot in the buffer
         for B in Page'Range loop
            SWD_Mailbox.Buffer (I * W25Q128.Page_Size + B) := Page (B);
         end loop;
      end loop;
   end Fill_Buffer;

begin
   --  start the clocks, timer and GPIO, and make the LED an output
   RP.Clock.Initialize (Pico.XOSC_Frequency);
   RP.Clock.Enable (RP.Clock.PERI);
   RP.Device.Timer.Enable;
   RP.GPIO.Enable;
   Pico.LED.Configure (RP.GPIO.Output);

   W25Q128.Initialize;

   --  fast blink and retry until the flash answers
   while not W25Q128.Is_Present loop
      Fault_Blink;
   end loop;

   --  serve requests forever
   loop
      if SWD_Mailbox.Request /= SWD_Mailbox.Done then
         Fill_Buffer;
         SWD_Mailbox.Done := SWD_Mailbox.Request;
         Pico.LED.Toggle;
      end if;
   end loop;
end Flash_Dump_SWD;
