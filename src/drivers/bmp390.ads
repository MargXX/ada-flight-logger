--  bmp390.ads
--  Bosch BMP390 barometer driver (I2C). The program must start the I2C bus
--  first. All procedures set Ok to False if an I2C transaction fails.
--
--  Enum VALUES mirror the datasheet field encodings, so the driver can shift
--  them straight into the register field.

with Byte_Types; use Byte_Types;
with Interfaces; use Interfaces;

package BMP390 is

   --  Power mode, PWR_CTRL.mode
   type Power_Mode is (Sleep, Forced, Normal);
   for Power_Mode use (Sleep => 0, Forced => 1, Normal => 3);

   --  Oversampling, OSR.osr_p and OSR.osr_t
   type Oversampling is (X1, X2, X4, X8, X16, X32);
   for Oversampling use
     (X1 => 0, X2 => 1, X4 => 2, X8 => 3, X16 => 4, X32 => 5);

   --  Output data rate, ODR.odr_sel
   type Output_Rate is
     (Rate_200_Hz, Rate_100_Hz, Rate_50_Hz, Rate_25_Hz, Rate_12_5_Hz);
   for Output_Rate use
     (Rate_200_Hz  => 0,
      Rate_100_Hz  => 1,
      Rate_50_Hz   => 2,
      Rate_25_Hz   => 3,
      Rate_12_5_Hz => 4);

   --  length of the factory calibration block
   Calibration_Size : constant := 21;

   --  Apply the settings and start measuring pressure and temperature.
   --  Ok is also False if the chip rejects the combination: the output period
   --  must be longer than the measurement time the oversampling needs.
   procedure Configure
     (Mode        : Power_Mode;
      Pressure    : Oversampling;
      Temperature : Oversampling;
      Rate        : Output_Rate;
      Ok          : out Boolean);

   --  Read the latest raw 24-bit values. Both are 0 if the read fails.
   --  Converting them to real units needs the calibration block.
   procedure Read_Raw
     (Pressure    : out Unsigned_32;
      Temperature : out Unsigned_32;
      Ok          : out Boolean);

   --  Read the factory calibration block. The log does not store it, so save
   --  it separately for each chip.
   procedure Read_Calibration
     (Data : out Byte_Array;
      Ok   : out Boolean)
     with Pre => Data'Length = Calibration_Size;

end BMP390;
