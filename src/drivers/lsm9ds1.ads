--  lsm9ds1.ads
--  ST LSM9DS1 accelerometer, gyroscope and magnetometer driver (I2C).
--  The program must start the I2C bus first. All procedures set Ok to False
--  if the I2C transaction fails.
--
--  Enum VALUES mirror the datasheet field encodings, so the driver can shift
--  them straight into the register field.

with Interfaces; use Interfaces;

package LSM9DS1 is

   --  Accel output rate, CTRL_REG6_XL.ODR_XL
   type Accel_Rate is
     (Accel_Off, Accel_10_Hz, Accel_50_Hz, Accel_119_Hz, Accel_238_Hz,
      Accel_476_Hz, Accel_952_Hz);
   for Accel_Rate use
     (Accel_Off    => 2#000#,
      Accel_10_Hz  => 2#001#,
      Accel_50_Hz  => 2#010#,
      Accel_119_Hz => 2#011#,
      Accel_238_Hz => 2#100#,
      Accel_476_Hz => 2#101#,
      Accel_952_Hz => 2#110#);

   --  Accel full scale, CTRL_REG6_XL.FS_XL. The encoding is not in size order.
   type Accel_Range is (Range_2_G, Range_16_G, Range_4_G, Range_8_G);
   for Accel_Range use
     (Range_2_G  => 2#00#,
      Range_16_G => 2#01#,
      Range_4_G  => 2#10#,
      Range_8_G  => 2#11#);

   --  Gyro output rate, CTRL_REG1_G.ODR_G
   type Gyro_Rate is
     (Gyro_Off, Gyro_14_9_Hz, Gyro_59_5_Hz, Gyro_119_Hz, Gyro_238_Hz,
      Gyro_476_Hz, Gyro_952_Hz);
   for Gyro_Rate use
     (Gyro_Off     => 2#000#,
      Gyro_14_9_Hz => 2#001#,
      Gyro_59_5_Hz => 2#010#,
      Gyro_119_Hz  => 2#011#,
      Gyro_238_Hz  => 2#100#,
      Gyro_476_Hz  => 2#101#,
      Gyro_952_Hz  => 2#110#);

   --  Gyro full scale, CTRL_REG1_G.FS_G
   type Gyro_Range is (Range_245_Dps, Range_500_Dps, Range_2000_Dps);
   for Gyro_Range use
     (Range_245_Dps  => 2#00#,
      Range_500_Dps  => 2#01#,
      Range_2000_Dps => 2#11#);

   --  Magnetometer mode, CTRL_REG3_M.MD
   type Mag_Mode is (Mag_Continuous, Mag_Off);
   for Mag_Mode use (Mag_Continuous => 2#00#, Mag_Off => 2#11#);

   --  one raw reading from any of the three sensors
   type Axes is record
      X, Y, Z : Integer_16 := 0;
   end record;

   --  Set the accelerometer's output rate and full scale.
   procedure Configure_Accel
     (Rate : Accel_Rate; Full_Scale : Accel_Range; Ok : out Boolean);

   --  Set the gyroscope's output rate and full scale.
   procedure Configure_Gyro
     (Rate : Gyro_Rate; Full_Scale : Gyro_Range; Ok : out Boolean);

   --  Turn the magnetometer on or off. Continuous also applies the settings
   --  used on the first flight: temperature compensation, XY ultra-high
   --  performance, 10 Hz.
   procedure Set_Mag_Mode (Mode : Mag_Mode; Ok : out Boolean);

   --  Read the latest accelerometer axes. Value is zeros if the read fails.
   procedure Read_Accel (Value : out Axes; Ok : out Boolean);

   --  Read the latest gyroscope axes. Value is zeros if the read fails.
   procedure Read_Gyro (Value : out Axes; Ok : out Boolean);

   --  Read the latest magnetometer axes. Value is zeros if the read fails.
   procedure Read_Mag (Value : out Axes; Ok : out Boolean);

end LSM9DS1;
