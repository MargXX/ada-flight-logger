--  flight_logger.adb
--  Flight firmware. Logs sensor data to flash in one fixed-period loop, with
--  no UART and no flash erases. Run flash_erase before each campaign.
--
--  General plan:
--  Every 10 ms tick reads the accelerometer to watch for launch.
--  Ground rate: logs a full sample every 10th tick (10 Hz).
--  Flight rate: logs every tick (100 Hz) for 10 minutes after launch, then
--  drops back and re-arms, so a false trigger only costs about 8 percent of
--  the flash. The page layout is in sample_log.ads.
--
--  LED: toggles on each page written. Fast blink is a boot fault, which keeps
--  retrying until it clears. Solid means the flash is full.

with Byte_Types; use Byte_Types;
with Interfaces; use Interfaces;

with BMP390;
with LSM9DS1;
with Launch_Detect;
with Pico;
with RP.Clock;
with RP.Device;
with RP.GPIO;    use RP.GPIO;
with RP.Timer;   use RP.Timer;
with Sample_Log;
with W25Q128;

procedure Flight_Logger is

   SDA : GPIO_Point := (Pin => 0);
   SCL : GPIO_Point := (Pin => 1);

   Tick_Us             : constant := 10_000;  --  100 Hz loop
   Ground_Log_Interval : constant := 10;      --  ticks between ground samples
   Flight_Rate_Ticks   : constant := 60_000;  --  10 minutes of 10 ms ticks

   Log         : Sample_Log.Log;
   Detector    : Launch_Detect.Detector;
   Next_Page   : Natural := 0;      --  next flash page to write
   Flight_Left : Natural := 0;      --  flight rate ticks left, 0 = ground
   Ground_Wait : Natural := 0;      --  ticks since the last ground sample
   Overruns    : Unsigned_32 := 0;  --  missed deadlines since boot
   Errors      : Unsigned_16 := 0;  --  failed reads and writes since boot
   Deadline    : Time := 0;         --  when the next tick is due

   --  Wait until the next tick is due. Each deadline comes from the previous
   --  one, so variation in the work does not build up as drift. If the
   --  deadline has already passed, this counts an overrun and returns.
   procedure Wait_For_Next_Tick is
   begin
      --  missed it: restart from now instead of rushing to catch up
      if Clock >= Deadline then
         Overruns := Overruns + 1;
         Deadline := Clock + Tick_Us;
         return;
      end if;

      --  on time: wait, then step from the old deadline so timing can't drift
      Busy_Wait_Until (Deadline);
      Deadline := Deadline + Tick_Us;
   end Wait_For_Next_Tick;

   --  Blink the LED fast for about a second to show a fault. Blocks while
   --  blinking, so the caller can loop on it and retry.
   procedure Fault_Blink is
   begin
      for I in 1 .. 20 loop
         Pico.LED.Toggle;
         RP.Device.Timer.Delay_Milliseconds (50);
      end loop;
   end Fault_Blink;

   --  Add one to the error count, stopping at the maximum so it can't wrap.
   procedure Count_Error is
   begin
      if Errors < Unsigned_16'Last then
         Errors := Errors + 1;
      end if;
   end Count_Error;

   --  Configure every sensor. Returns False at the first failure.
   function Sensors_Configured return Boolean is
      use BMP390;
      use LSM9DS1;
      Ok : Boolean;
   begin
      Configure_Accel (Accel_119_Hz, Range_16_G, Ok);
      if Ok then
         Configure_Gyro (Gyro_119_Hz, Range_2000_Dps, Ok);
      end if;

      --  not logged, so power it down
      if Ok then
         Set_Mag_Mode (Mag_Off, Ok);
      end if;

      --  50 Hz because this oversampling needs more than the 200 Hz period
      if Ok then
         BMP390.Configure (Normal, X4, X1, Rate_50_Hz, Ok);
      end if;
      return Ok;
   end Sensors_Configured;

   --  Returns False if flash holds data from a different format, which means
   --  it still needs erasing. A blank chip passes.
   function Flash_Format_Matches return Boolean is
      Version  : Byte_Array (1 .. 1);
      Sequence : Byte_Array (1 .. 4);
   begin
      W25Q128.Read (Sample_Log.Sequence_Offset, Sequence);
      W25Q128.Read (Sample_Log.Version_Offset, Version);
      return Sample_Log.Is_Blank (Sequence)
        or else Version (1) = Sample_Log.Format_Version;
   end Flash_Format_Matches;

   --  Point Next_Page at the first blank page, so a brownout restart carries
   --  on after the existing data instead of overwriting it.
   procedure Find_Next_Page is
      Sequence : Byte_Array (1 .. 4);
   begin
      while Next_Page < W25Q128.Page_Count loop
         --  only read the sequence bytes to keep the scan quick
         W25Q128.Read
           (Next_Page * W25Q128.Page_Size + Sample_Log.Sequence_Offset,
            Sequence);
         exit when Sample_Log.Is_Blank (Sequence);
         Next_Page := Next_Page + 1;
      end loop;
   end Find_Next_Page;

   --  Turn the LED on and stop. Never returns. Stopping protects existing
   --  data, because nothing here can overwrite it.
   procedure Halt_Flash_Full is
   begin
      Pico.LED.Set;
      loop
         null;
      end loop;
   end Halt_Flash_Full;

   --  Write the current page to flash and move on to the next one.
   --  Halts if that was the last page.
   procedure Write_Page is
      Ok : Boolean;
   begin
      Sample_Log.Finish_Page (Log, Overruns, Errors, Flight_Left > 0);
      W25Q128.Program_Page
        (Next_Page * W25Q128.Page_Size, Sample_Log.Page (Log), Ok);
      if not Ok then
         Count_Error;
      end if;

      --  heartbeat: one LED change per page
      Pico.LED.Toggle;

      Next_Page := Next_Page + 1;
      Sample_Log.Next_Page (Log);

      if Next_Page = W25Q128.Page_Count then
         Halt_Flash_Full;
      end if;
   end Write_Page;

   --  One pass of the loop: check for launch, then log a sample if this tick
   --  is due for one. A failed read logs zeros and adds to the error count.
   procedure Tick is
      --  the log keeps the lower 32 bits, which wrap about every 71 minutes
      Timestamp : constant Unsigned_32 :=
        Unsigned_32 (Clock and 16#FFFF_FFFF#);
      Accel     : LSM9DS1.Axes;
      Gyro      : LSM9DS1.Axes;
      Pressure  : Unsigned_32;
      Temp      : Unsigned_32;
      Ok        : Boolean;
      Launched  : Boolean := False;
   begin
      --  the accelerometer is read every tick to watch for launch
      LSM9DS1.Read_Accel (Accel, Ok);
      if Ok then
         Launch_Detect.Update (Detector, Accel.X, Accel.Y, Accel.Z, Launched);
      else
         Count_Error;
      end if;

      --  count down flight rate, or start it if launch was just detected
      if Flight_Left > 0 then
         Flight_Left := Flight_Left - 1;
         if Flight_Left = 0 then
            Launch_Detect.Reset (Detector);  --  re-arm
         end if;
      elsif Launched then
         Flight_Left := Flight_Rate_Ticks;
      end if;

      --  at ground rate, skip this tick unless a sample is due
      if Flight_Left = 0 then
         Ground_Wait := Ground_Wait + 1;
         if Ground_Wait < Ground_Log_Interval then
            return;
         end if;
         Ground_Wait := 0;
      end if;

      --  read the remaining sensors only when logging
      LSM9DS1.Read_Gyro (Gyro, Ok);
      if not Ok then
         Count_Error;
      end if;
      BMP390.Read_Raw (Pressure, Temp, Ok);
      if not Ok then
         Count_Error;
      end if;

      Sample_Log.Add
        (Log,
         (Timestamp   => Timestamp,
          Accel_X     => Accel.X,
          Accel_Y     => Accel.Y,
          Accel_Z     => Accel.Z,
          Gyro_X      => Gyro.X,
          Gyro_Y      => Gyro.Y,
          Gyro_Z      => Gyro.Z,
          Pressure    => Pressure,
          Temperature => Temp));

      if Sample_Log.Is_Full (Log) then
         Write_Page;
      end if;
   end Tick;

begin
   --  start the clocks, timer and GPIO, and make the LED an output
   RP.Clock.Initialize (Pico.XOSC_Frequency);
   RP.Clock.Enable (RP.Clock.PERI);
   RP.Device.Timer.Enable;
   RP.GPIO.Enable;
   Pico.LED.Configure (RP.GPIO.Output);

   --  start the sensor bus at 100 kHz
   --  Schmitt is required or the bus does not drive at all
   SDA.Configure (Output, Pull_Up, RP.GPIO.I2C, Schmitt => True);
   SCL.Configure (Output, Pull_Up, RP.GPIO.I2C, Schmitt => True);
   RP.Device.I2CM_0.Configure (Baudrate => 100_000);

   W25Q128.Initialize;

   --  fast blink and retry until flash is usable
   while not (W25Q128.Is_Present and then Flash_Format_Matches) loop
      Fault_Blink;
   end loop;

   --  fast blink and retry until the sensors respond
   while not Sensors_Configured loop
      Fault_Blink;
   end loop;

   --  pick up after any existing data
   Find_Next_Page;
   if Next_Page = W25Q128.Page_Count then
      Halt_Flash_Full;
   end if;
   Sample_Log.Start (Log, Unsigned_32 (Next_Page));

   --  main loop: do the work, then wait for the next 10 ms deadline
   Deadline := Clock + Tick_Us;
   loop
      Tick;
      Wait_For_Next_Tick;
   end loop;
end Flight_Logger;
