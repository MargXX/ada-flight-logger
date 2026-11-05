--  sensor_test.adb
--  Bench test. Checks the flash ID and sensor setup, prints the barometer
--  calibration, then prints sensor readings over UART every 500 ms.
--  Does not write to flash. LED toggles after each set of readings.

with Byte_Types; use Byte_Types;
with Interfaces; use Interfaces;

with BMP390;
with Debug_UART; use Debug_UART;
with LSM9DS1;
with Pico;
with RP.Clock;
with RP.Device;
with RP.GPIO;    use RP.GPIO;
with W25Q128;

procedure Sensor_Test is

   SDA : GPIO_Point := (Pin => 0);
   SCL : GPIO_Point := (Pin => 1);

   --  Returns V as text. Goes through Integer_32 because Integer_16'Image
   --  hangs on this runtime.
   function Image (V : Integer_16) return String is
     (Integer_32'Image (Integer_32 (V)));

   --  Print one setup step's result.
   procedure Report_Setup (Label : String; Ok : Boolean) is
   begin
      Put_Line (Label & (if Ok then ": ok" else ": FAILED"));
   end Report_Setup;

   --  Print one reading, or a failure message if the read failed.
   procedure Print_Axes
     (Label : String; Value : LSM9DS1.Axes; Ok : Boolean) is
   begin
      if Ok then
         Put_Line (Label & " X=" & Image (Value.X) & " Y=" & Image (Value.Y)
                   & " Z=" & Image (Value.Z));
      else
         Put_Line (Label & " read FAILED");
      end if;
   end Print_Axes;

   Ok          : Boolean;
   Axes        : LSM9DS1.Axes;
   Pressure    : Unsigned_32;
   Temperature : Unsigned_32;
   Calibration : Byte_Array (1 .. BMP390.Calibration_Size);

begin
   --  start the clocks, timer and GPIO, and make the LED an output
   RP.Clock.Initialize (Pico.XOSC_Frequency);
   RP.Clock.Enable (RP.Clock.PERI);
   RP.Device.Timer.Enable;
   RP.GPIO.Enable;
   Pico.LED.Configure (RP.GPIO.Output);

   Debug_UART.Initialize;

   --  start the sensor bus at 100 kHz
   --  Schmitt is required or the bus does not drive at all
   SDA.Configure (Output, Pull_Up, RP.GPIO.I2C, Schmitt => True);
   SCL.Configure (Output, Pull_Up, RP.GPIO.I2C, Schmitt => True);
   RP.Device.I2CM_0.Configure (Baudrate => 100_000);

   W25Q128.Initialize;

   Put_Line ("--- Sensor Test ---");
   Report_Setup ("Flash ID", W25Q128.Is_Present);

   --  same settings as the flight logger, plus the magnetometer
   LSM9DS1.Configure_Accel (LSM9DS1.Accel_119_Hz, LSM9DS1.Range_16_G, Ok);
   Report_Setup ("Accel setup", Ok);
   LSM9DS1.Configure_Gyro (LSM9DS1.Gyro_119_Hz, LSM9DS1.Range_2000_Dps, Ok);
   Report_Setup ("Gyro setup", Ok);
   LSM9DS1.Set_Mag_Mode (LSM9DS1.Mag_Continuous, Ok);
   Report_Setup ("Mag setup", Ok);
   BMP390.Configure (BMP390.Normal, BMP390.X4, BMP390.X1,
                     BMP390.Rate_50_Hz, Ok);
   Report_Setup ("Baro setup", Ok);

   --  the log doesn't store this, and it's needed to convert raw baro values
   BMP390.Read_Calibration (Calibration, Ok);
   if Ok then
      Put_Line ("Baro calibration, save this to convert logs:");
      Put_Hex_Line (Calibration);
   else
      Put_Line ("Baro calibration read FAILED");
   end if;

   --  print every sensor, then wait
   loop
      LSM9DS1.Read_Accel (Axes, Ok);
      Print_Axes ("Accel", Axes, Ok);
      LSM9DS1.Read_Gyro (Axes, Ok);
      Print_Axes ("Gyro ", Axes, Ok);
      LSM9DS1.Read_Mag (Axes, Ok);
      Print_Axes ("Mag  ", Axes, Ok);

      BMP390.Read_Raw (Pressure, Temperature, Ok);
      if Ok then
         Put_Line ("Baro  P=" & Integer_32'Image (Integer_32 (Pressure))
                   & " T=" & Integer_32'Image (Integer_32 (Temperature)));
      else
         Put_Line ("Baro  read FAILED");
      end if;

      Put_Line ("");
      Pico.LED.Toggle;
      RP.Device.Timer.Delay_Milliseconds (500);
   end loop;
end Sensor_Test;
