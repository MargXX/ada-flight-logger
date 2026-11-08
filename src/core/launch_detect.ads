--  launch_detect.ads
--  Launch detection from raw accelerometer counts.
--
--  General plan:
--  Launch is declared after Required_Ticks consecutive readings above the
--  threshold, so one bump cannot trigger it. No hardware access, so this can
--  be tested on a host machine.
--
--  Why 3 g: in the April 2026 log, sampled at about 5 Hz, boost was the only
--  sustained stretch above 3 g in 25 hours.

with Interfaces; use Interfaces;

package Launch_Detect
  with Pure
is
   --  only valid at the "+/-16 g" setting: 0.732 mg/LSB is about 1366
   --  counts/g, so the full count range is about +/-24 g. Checked against
   --  gravity on the bench: 1314 counts at rest.
   Counts_Per_G     : constant := 1366;
   Threshold_G      : constant := 3;
   Threshold_Counts : constant := Threshold_G * Counts_Per_G;

   --  at the 10 ms tick this means 100 ms above the threshold
   Required_Ticks   : constant := 10;

   --  holds the length of the current run of high readings
   type Detector is private;

   --  Feed in one accelerometer reading, in raw counts.
   --  Launched is True once the run reaches Required_Ticks, and stays True
   --  while readings stay high. A reading at or below the threshold restarts
   --  the run. The full Integer_16 range is safe.
   procedure Update
     (D        : in out Detector;
      X, Y, Z  : Integer_16;
      Launched : out Boolean);

   --  Clear the run so detection starts over.
   procedure Reset (D : in out Detector);

private
   type Detector is record
      Run : Natural := 0;  --  consecutive readings above the threshold
   end record;
end Launch_Detect;
