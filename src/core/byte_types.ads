--  byte_types.ads
--  Plain byte types shared by the flash driver and the log format.
--  Kept apart from the HAL types so src/core can build on a host machine.

with Interfaces;

package Byte_Types
  with Pure
is
   --  one raw byte, with the Interfaces shift and bitwise operators
   subtype Byte is Interfaces.Unsigned_8;

   --  any length of raw bytes, such as a flash page
   type Byte_Array is array (Natural range <>) of Byte;
end Byte_Types;
