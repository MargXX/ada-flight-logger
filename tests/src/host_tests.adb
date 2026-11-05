with Ada.Command_Line;
with Ada.Text_IO; use Ada.Text_IO;
with Byte_Types;  use Byte_Types;
with Interfaces;  use Interfaces;
with Launch_Detect;
with Sample_Log;

--  host_tests.adb
--  Tests the code with no hardware access, on the computer instead of the
--  Pico. Exits with a failure status if any check fails.

procedure Host_Tests is

   Failures : Natural := 0;

   --  Print a pass or FAIL line for one check and count failures.
   procedure Check (Name : String; Condition : Boolean) is
   begin
      Put_Line ((if Condition then "pass  " else "FAIL  ") & Name);
      if not Condition then
         Failures := Failures + 1;
      end if;
   end Check;

   --  Check the trigger, run restart, re-arm and extreme input behavior.
   procedure Test_Launch_Detect is
      use Launch_Detect;
      D        : Detector;
      Launched : Boolean := False;
      High     : constant Integer_16 := Integer_16 (Threshold_Counts + 1);
   begin
      for I in 1 .. Required_Ticks - 1 loop
         Update (D, High, 0, 0, Launched);
      end loop;
      Check ("no trigger one reading short", not Launched);

      Update (D, 0, 0, 1366, Launched);
      Check ("1 g on the pad does not trigger", not Launched);

      for I in 1 .. Required_Ticks - 1 loop
         Update (D, 0, High, 0, Launched);
      end loop;
      Check ("a low reading restarts the run", not Launched);

      Update (D, 0, High, 0, Launched);
      Check ("full run triggers", Launched);

      Update (D, 0, High, 0, Launched);
      Check ("stays triggered during the run", Launched);

      Reset (D);
      Update (D, High, 0, 0, Launched);
      Check ("reset re-arms", not Launched);

      Reset (D);
      Update (D, Integer_16'First, Integer_16'First, Integer_16'First,
              Launched);
      Check ("largest magnitude does not overflow", not Launched);

      Reset (D);
      Update (D, Integer_16 (Threshold_Counts), 0, 0, Launched);
      for I in 2 .. Required_Ticks loop
         Update (D, Integer_16 (Threshold_Counts), 0, 0, Launched);
      end loop;
      Check ("exactly at the threshold does not trigger", not Launched);
   end Test_Launch_Detect;

   --  Check every field's position and byte order against the layout.
   procedure Test_Sample_Log is
      use Sample_Log;
      L : Log;
      S : constant Sample :=
        (Timestamp   => 16#0403_0201#,
         Accel_X     => 16#0605#,
         Accel_Y     => -2,
         Accel_Z     => 16#0A09#,
         Gyro_X      => 16#0C0B#,
         Gyro_Y      => 16#0E0D#,
         Gyro_Z      => Integer_16'First,
         Pressure    => 16#131211#,
         Temperature => 16#161514#);
   begin
      Start (L, 16#2221_201F#);
      Check ("new page is not full", not Is_Full (L));

      for I in 1 .. Samples_Per_Page loop
         Check ("not full before sample" & I'Image, not Is_Full (L));
         Add (L, S);
      end loop;
      Check ("full after 11 samples", Is_Full (L));

      Finish_Page (L, 16#3433_3231#, 16#3635#, Flight_Rate => True);

      declare
         P : constant Page_Buffer := Page (L);
      begin
         Check ("timestamp is LSB first",
                P (0 .. 3) = Byte_Array'(16#01#, 16#02#, 16#03#, 16#04#));
         Check ("accel X", P (4 .. 5) = Byte_Array'(16#05#, 16#06#));
         Check ("negative accel Y", P (6 .. 7) = Byte_Array'(16#FE#, 16#FF#));
         Check ("gyro Z minimum",
                P (14 .. 15) = Byte_Array'(16#00#, 16#80#));
         Check ("pressure",
                P (16 .. 18) = Byte_Array'(16#11#, 16#12#, 16#13#));
         Check ("temperature",
                P (19 .. 21) = Byte_Array'(16#14#, 16#15#, 16#16#));
         Check ("second sample starts at byte 22", P (22) = 16#01#);
         Check ("last sample starts at byte 220", P (220) = 16#01#);
         Check ("overruns",
                P (242 .. 245) = Byte_Array'(16#31#, 16#32#, 16#33#, 16#34#));
         Check ("errors", P (246 .. 247) = Byte_Array'(16#35#, 16#36#));
         Check ("flight rate flag", P (248) = 1);
         Check ("reserved bytes", P (249 .. 250) = Byte_Array'(0, 0));
         Check ("format version", P (251) = Format_Version);
         Check ("sequence",
                P (252 .. 255) = Byte_Array'(16#1F#, 16#20#, 16#21#, 16#22#));
         Check ("written page is not blank",
                not Is_Blank (P (Sequence_Offset .. P'Last)));
      end;

      Next_Page (L);
      Check ("next page is empty", not Is_Full (L));
      Finish_Page (L, 0, 0, Flight_Rate => False);
      declare
         P : constant Page_Buffer := Page (L);
      begin
         Check ("sequence advances", P (252) = 16#20#);
         Check ("ground rate flag", P (248) = 0);
         Check ("old samples cleared", P (0) = 0);
      end;

      Check ("erased bytes are blank",
             Is_Blank (Byte_Array'(16#FF#, 16#FF#, 16#FF#, 16#FF#)));
   end Test_Sample_Log;

begin
   Test_Launch_Detect;
   Test_Sample_Log;

   New_Line;
   if Failures = 0 then
      Put_Line ("All tests passed");
   else
      Put_Line ("Failures:" & Failures'Image);
      Ada.Command_Line.Set_Exit_Status (Ada.Command_Line.Failure);
   end if;
end Host_Tests;
