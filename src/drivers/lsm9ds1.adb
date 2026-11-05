with Ada.Unchecked_Conversion;
with HAL;     use HAL;
with HAL.I2C; use HAL.I2C;
with RP.Device;

package body LSM9DS1 is

   --  8-bit address form, because rp2040_hal shifts right internally
   Addr_AG  : constant I2C_Address := 16#D6#;
   Addr_Mag : constant I2C_Address := 16#3C#;

   CTRL_REG1_G  : constant UInt8 := 16#10#;
   OUT_X_L_G    : constant UInt8 := 16#18#;
   CTRL_REG6_XL : constant UInt8 := 16#20#;
   OUT_X_L_XL   : constant UInt8 := 16#28#;
   CTRL_REG1_M  : constant UInt8 := 16#20#;
   CTRL_REG3_M  : constant UInt8 := 16#22#;

   --  OUT_X_L_M with bit 7 set, which auto-increments multi-byte reads
   OUT_X_L_M_Multi : constant UInt8 := 16#A8#;

   --  temperature compensation, XY ultra-high performance, 10 Hz
   Mag_Settings : constant UInt8 := 2#1111_0000#;

   --  short so a stuck bus can't stall the logging loop for long
   Timeout_Ms : constant := 5;

   --  keeps the same bits, so two's complement values come out signed
   function To_I16 is new Ada.Unchecked_Conversion (Unsigned_16, Integer_16);

   --  Write one register. Ok is False if the transaction fails.
   procedure Write
     (Addr : I2C_Address; Reg : UInt8; Value : UInt8; Ok : out Boolean)
   is
      Status : I2C_Status;
   begin
      RP.Device.I2CM_0.Mem_Write
        (Addr          => Addr,
         Mem_Addr      => UInt16 (Reg),
         Mem_Addr_Size => Memory_Size_8b,
         Data          => I2C_Data'(1 => Value),
         Status        => Status,
         Timeout       => Timeout_Ms);
      Ok := Status = HAL.I2C.Ok;
   end Write;

   --  Read 3 axes starting at Reg. Value is zeros if the read fails.
   procedure Read_Axes
     (Addr  : I2C_Address;
      Reg   : UInt8;
      Value : out Axes;
      Ok    : out Boolean)
   is
      Data   : I2C_Data (1 .. 6);
      Status : I2C_Status;

      --  join one axis's two bytes, which arrive low byte first
      function Axis (Low_Index : Positive) return Integer_16 is
        (To_I16 (Unsigned_16 (Data (Low_Index))
                 + Unsigned_16 (Data (Low_Index + 1)) * 256));
   begin
      Value := (others => 0);

      --  read all 6 bytes in one transaction
      RP.Device.I2CM_0.Mem_Read
        (Addr          => Addr,
         Mem_Addr      => UInt16 (Reg),
         Mem_Addr_Size => Memory_Size_8b,
         Data          => Data,
         Status        => Status,
         Timeout       => Timeout_Ms);
      Ok := Status = HAL.I2C.Ok;

      if Ok then
         Value := (X => Axis (1), Y => Axis (3), Z => Axis (5));
      end if;
   end Read_Axes;

   --  Set the accelerometer's output rate and full scale.
   procedure Configure_Accel
     (Rate : Accel_Rate; Full_Scale : Accel_Range; Ok : out Boolean)
   is
      --  ODR_XL is bits 7:5 and FS_XL is bits 4:3
      Value : constant UInt8 :=
        UInt8 (Accel_Rate'Enum_Rep (Rate)) * 32
        + UInt8 (Accel_Range'Enum_Rep (Full_Scale)) * 8;
   begin
      Write (Addr_AG, CTRL_REG6_XL, Value, Ok);
   end Configure_Accel;

   --  Set the gyroscope's output rate and full scale.
   procedure Configure_Gyro
     (Rate : Gyro_Rate; Full_Scale : Gyro_Range; Ok : out Boolean)
   is
      --  ODR_G is bits 7:5 and FS_G is bits 4:3
      Value : constant UInt8 :=
        UInt8 (Gyro_Rate'Enum_Rep (Rate)) * 32
        + UInt8 (Gyro_Range'Enum_Rep (Full_Scale)) * 8;
   begin
      Write (Addr_AG, CTRL_REG1_G, Value, Ok);
   end Configure_Gyro;

   --  Turn the magnetometer on or off.
   procedure Set_Mag_Mode (Mode : Mag_Mode; Ok : out Boolean) is
   begin
      Ok := True;

      --  apply the settings before starting continuous conversions
      if Mode = Mag_Continuous then
         Write (Addr_Mag, CTRL_REG1_M, Mag_Settings, Ok);
      end if;

      if Ok then
         Write (Addr_Mag, CTRL_REG3_M, UInt8 (Mag_Mode'Enum_Rep (Mode)), Ok);
      end if;
   end Set_Mag_Mode;

   --  Read the latest accelerometer axes.
   procedure Read_Accel (Value : out Axes; Ok : out Boolean) is
   begin
      Read_Axes (Addr_AG, OUT_X_L_XL, Value, Ok);
   end Read_Accel;

   --  Read the latest gyroscope axes.
   procedure Read_Gyro (Value : out Axes; Ok : out Boolean) is
   begin
      Read_Axes (Addr_AG, OUT_X_L_G, Value, Ok);
   end Read_Gyro;

   --  Read the latest magnetometer axes.
   procedure Read_Mag (Value : out Axes; Ok : out Boolean) is
   begin
      Read_Axes (Addr_Mag, OUT_X_L_M_Multi, Value, Ok);
   end Read_Mag;

end LSM9DS1;
