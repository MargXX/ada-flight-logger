--  w25q128.ads
--  Winbond W25Q128 16 MB flash driver (SPI) on GP16-GP19.

with Byte_Types; use Byte_Types;

package W25Q128 is

   Page_Size  : constant := 256;
   Page_Count : constant := 65_536;

   --  How much to erase. Each size has its own flash command.
   type Erase_Size is (Sector_4_KB, Block_32_KB, Block_64_KB, Whole_Chip);

   subtype Flash_Address is Natural range 0 .. Page_Count * Page_Size - 1;

   --  Set up the pins and the SPI port. Call before anything else here.
   procedure Initialize;

   --  Returns True if the chip answers with the W25Q128 JEDEC ID.
   function Is_Present return Boolean;

   --  Fill Data with the bytes starting at Address. Can cross page boundaries.
   procedure Read (Address : Flash_Address; Data : out Byte_Array);

   --  Write one full page and wait for it to finish.
   --  Address must be the start of a page, and the page must already be
   --  erased. Ok is False if the chip does not finish in time.
   procedure Program_Page
     (Address : Flash_Address;
      Data    : Byte_Array;
      Ok      : out Boolean)
     with Pre => Data'Length = Page_Size and then Address mod Page_Size = 0;

   --  Erase and block until it finishes, which can take minutes for
   --  Whole_Chip. Address is ignored for Whole_Chip. Ok is False if the chip
   --  does not finish in time.
   procedure Erase
     (Size    : Erase_Size;
      Address : Flash_Address;
      Ok      : out Boolean);

end W25Q128;
