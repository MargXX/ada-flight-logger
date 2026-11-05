package body Launch_Detect is

   --  Square one axis. Largest result is 2**30, so three of them still fit
   --  in Unsigned_32 when added together.
   function Square (V : Integer_16) return Unsigned_32 is
      --  widen first so the multiply can't overflow
      W : constant Integer_32 := Integer_32 (V);
   begin
      return Unsigned_32 (W * W);
   end Square;

   --  compare squared values so the M0+ never has to take a square root
   Threshold_Squared : constant Unsigned_32 :=
     Unsigned_32 (Threshold_Counts) * Unsigned_32 (Threshold_Counts);

   --  Feed in one reading and report whether the run has reached launch.
   procedure Update
     (D        : in out Detector;
      X, Y, Z  : Integer_16;
      Launched : out Boolean)
   is
      Magnitude_Squared : constant Unsigned_32 :=
        Square (X) + Square (Y) + Square (Z);
   begin
      if Magnitude_Squared > Threshold_Squared then
         --  extend the run, capped so the counter can never overflow
         D.Run := Natural'Min (D.Run + 1, Required_Ticks);
      else
         --  any low reading restarts the run
         D.Run := 0;
      end if;

      Launched := D.Run = Required_Ticks;
   end Update;

   --  Clear the run so detection starts over.
   procedure Reset (D : in out Detector) is
   begin
      D.Run := 0;
   end Reset;

end Launch_Detect;
