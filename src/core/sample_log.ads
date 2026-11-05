--  sample_log.ads
--  Packs samples into 256-byte flash pages. No hardware access, so this can
--  be tested on a host machine.
--
--  Logging layout, all values LSB first:
--
--  Sample, 22 bytes:
--     0-3    Timestamp, microseconds since boot, 32-bit
--     4-9    Accel X, Y, Z, signed 16-bit raw
--    10-15   Gyro X, Y, Z, signed 16-bit raw
--    16-18   Pressure, BMP390 raw 24-bit
--    19-21   Temperature, BMP390 raw 24-bit
--
--  Page, 256 bytes:
--     0-241  11 samples
--   242-245  Missed tick deadlines since boot, 32-bit
--   246-247  Failed sensor reads or flash writes since boot, 16-bit
--   248      Rate when the page was written: 0 ground, 1 flight
--   249-250  Reserved, zero
--   251      Format version
--   252-255  Page sequence number, 32-bit. All 0xFF means never written.

with Byte_Types; use Byte_Types;
with Interfaces; use Interfaces;

package Sample_Log
  with Pure
is
   Page_Size        : constant := 256;
   Sample_Size      : constant := 22;
   Samples_Per_Page : constant := 11;
   Version_Offset   : constant := 251;
   Sequence_Offset  : constant := 252;

   --  change this whenever the layout above changes
   Format_Version   : constant Byte := 16#A2#;

   subtype Page_Buffer is Byte_Array (0 .. Page_Size - 1);

   --  one full reading, before packing
   type Sample is record
      Timestamp   : Unsigned_32 := 0;  --  microseconds since boot
      Accel_X     : Integer_16 := 0;
      Accel_Y     : Integer_16 := 0;
      Accel_Z     : Integer_16 := 0;
      Gyro_X      : Integer_16 := 0;
      Gyro_Y      : Integer_16 := 0;
      Gyro_Z      : Integer_16 := 0;
      Pressure    : Unsigned_32 := 0;  --  raw 24-bit
      Temperature : Unsigned_32 := 0;  --  raw 24-bit
   end record;

   --  the page being filled and its sequence number
   type Log is private;

   --  Empty the page and set its sequence number. Call before the first Add.
   --  Sequence should equal the page's index in flash.
   procedure Start (L : in out Log; Sequence : Unsigned_32);

   --  Pack one sample into the next open slot. The page must not be full.
   procedure Add (L : in out Log; S : Sample);

   --  Returns True when the page holds Samples_Per_Page samples.
   function Is_Full (L : Log) return Boolean;

   --  Write the page's trailer. The page is then ready to go to flash.
   procedure Finish_Page
     (L           : in out Log;
      Overruns    : Unsigned_32;
      Errors      : Unsigned_16;
      Flight_Rate : Boolean);

   --  Returns the current contents of the page.
   function Page (L : Log) return Page_Buffer;

   --  Empty the page and advance the sequence number by one.
   procedure Next_Page (L : in out Log);

   --  Returns True if the 4 bytes at Sequence_Offset are all 0xFF, which
   --  means that page was never written since the last erase.
   function Is_Blank (Sequence_Bytes : Byte_Array) return Boolean;

private
   type Log is record
      Buffer   : Page_Buffer := (others => 0);
      Count    : Natural := 0;      --  samples in the page so far
      Sequence : Unsigned_32 := 0;  --  stamped into the trailer
   end record;
end Sample_Log;
