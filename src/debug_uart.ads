--  debug_uart.ads
--  UART1 for the bench tools, on GP8 TX and GP9 RX at 115200 baud.
--  The flight logger does not use this package.

with Byte_Types; use Byte_Types;

package Debug_UART is

   --  Set up the pins and the UART. Call before anything else here.
   procedure Initialize;

   --  Send text with no line ending. Blocks until it is all in the TX FIFO.
   procedure Put (Text : String);

   --  Send text followed by CR LF.
   procedure Put_Line (Text : String);

   --  Send the bytes as one line of uppercase hex, 2 characters per byte.
   procedure Put_Hex_Line (Data : Byte_Array);

   --  Block until the given character arrives on RX.
   procedure Wait_For (C : Character);

end Debug_UART;
