with Ada.Unchecked_Conversion;

package body Sample_Log is

   --  keeps the same bits, so negative values pack as two's complement
   function To_U16 is new Ada.Unchecked_Conversion (Integer_16, Unsigned_16);

   --  Store the lowest Size bytes of Value at Offset, LSB first.
   procedure Put_LE
     (Buffer : in out Page_Buffer;
      Offset : Natural;
      Value  : Unsigned_32;
      Size   : Positive)
   is
      V : Unsigned_32 := Value;
   begin
      for I in 0 .. Size - 1 loop
         --  take the low byte, then move the next one down
         Buffer (Offset + I) := Byte (V and 16#FF#);
         V := Shift_Right (V, 8);
      end loop;
   end Put_LE;

   --  Store a signed 16-bit value at Offset, LSB first.
   procedure Put_I16
     (Buffer : in out Page_Buffer;
      Offset : Natural;
      Value  : Integer_16) is
   begin
      Put_LE (Buffer, Offset, Unsigned_32 (To_U16 (Value)), 2);
   end Put_I16;

   --  Empty the page and set its sequence number.
   procedure Start (L : in out Log; Sequence : Unsigned_32) is
   begin
      L.Buffer := (others => 0);
      L.Count := 0;
      L.Sequence := Sequence;
   end Start;

   --  Pack one sample into the next open slot.
   procedure Add (L : in out Log; S : Sample) is
      --  samples sit back to back from the start of the page
      Base : constant Natural := L.Count * Sample_Size;
   begin
      Put_LE (L.Buffer, Base, S.Timestamp, 4);
      Put_I16 (L.Buffer, Base + 4, S.Accel_X);
      Put_I16 (L.Buffer, Base + 6, S.Accel_Y);
      Put_I16 (L.Buffer, Base + 8, S.Accel_Z);
      Put_I16 (L.Buffer, Base + 10, S.Gyro_X);
      Put_I16 (L.Buffer, Base + 12, S.Gyro_Y);
      Put_I16 (L.Buffer, Base + 14, S.Gyro_Z);
      Put_LE (L.Buffer, Base + 16, S.Pressure, 3);
      Put_LE (L.Buffer, Base + 19, S.Temperature, 3);
      L.Count := L.Count + 1;
   end Add;

   --  Returns True when no sample slots are left.
   function Is_Full (L : Log) return Boolean is
     (L.Count = Samples_Per_Page);

   --  Write the trailer, following the layout in the spec.
   procedure Finish_Page
     (L           : in out Log;
      Overruns    : Unsigned_32;
      Errors      : Unsigned_16;
      Flight_Rate : Boolean) is
   begin
      Put_LE (L.Buffer, 242, Overruns, 4);
      Put_LE (L.Buffer, 246, Unsigned_32 (Errors), 2);
      L.Buffer (248) := (if Flight_Rate then 1 else 0);
      L.Buffer (Version_Offset) := Format_Version;
      Put_LE (L.Buffer, Sequence_Offset, L.Sequence, 4);
   end Finish_Page;

   --  Returns the current contents of the page.
   function Page (L : Log) return Page_Buffer is (L.Buffer);

   --  Empty the page and advance the sequence number.
   procedure Next_Page (L : in out Log) is
   begin
      L.Buffer := (others => 0);
      L.Count := 0;
      L.Sequence := L.Sequence + 1;
   end Next_Page;

   --  Returns True if every byte is 0xFF, the erased flash value.
   function Is_Blank (Sequence_Bytes : Byte_Array) return Boolean is
     (for all B of Sequence_Bytes => B = 16#FF#);

end Sample_Log;
