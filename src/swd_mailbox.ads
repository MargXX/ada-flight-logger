--  swd_mailbox.ads
--  Shared memory for flash_dump_swd. The debugger writes a request here over
--  SWD, the Pico fills Buffer from flash, and the debugger reads it back.
--  Exported names give fixed symbols to look up with nm.
--
--  Handshake:
--  1. Debugger writes Start_Page and Page_Count, then bumps Request.
--  2. Pico reads the pages into Buffer, then sets Done to Request.

with Byte_Types; use Byte_Types;
with Interfaces; use Interfaces;

package SWD_Mailbox is

   --  pages per request, 128 KB of RAM
   Max_Pages : constant := 512;

   Start_Page : Unsigned_32 := 0
     with Volatile, Export, External_Name => "swd_start_page";
   Page_Count : Unsigned_32 := 0
     with Volatile, Export, External_Name => "swd_page_count";
   Request    : Unsigned_32 := 0
     with Volatile, Export, External_Name => "swd_request";
   Done       : Unsigned_32 := 0
     with Volatile, Export, External_Name => "swd_done";

   Buffer : Byte_Array (0 .. Max_Pages * 256 - 1) := (others => 0)
     with Volatile, Export, External_Name => "swd_buffer";

end SWD_Mailbox;
