%%%-------------------------------------------------------------------
%%% @doc LCD1602 driver over I2C (PCF8574 backpack).
%%%
%%% Drives a standard 16x2 HD44780 LCD via the common PCF8574 I2C
%%% I/O expander backpack. Uses AtomVM's built-in `i2c' driver, so it
%%% can share an open I2C bus with the sensor modules.
%%%
%%% The PCF8574 chip sits between the I2C bus and the LCD controller.
%%% You only wire 4 lines to the ESP32 (SDA, SCL, VCC, GND). The
%%% backpack PCB has the PCF8574's 8 output pins (P0-P7) hardwired
%%% to the HD44780 LCD control and data pins:
%%%
%%%   PCF8574 pin    HD44780 pin    Purpose
%%%   ───────────    ───────────    ────────────────────────────────
%%%   P0             RS             Register Select (0=command, 1=data)
%%%   P1             RW             Read/Write (always 0 for write)
%%%   P2             EN             Enable — pulse high to latch data
%%%   P3             Backlight      Controls the LED backlight
%%%   P4             D4             Data bit 4 ┐
%%%   P5             D5             Data bit 5 │ 4-bit data bus
%%%   P6             D6             Data bit 6 │
%%%   P7             D7             Data bit 7 ┘
%%%
%%% Each I2C write sends one byte that sets all 8 PCF8574 outputs
%%% simultaneously. We bit-bang the HD44780 protocol by composing
%%% bytes with the right control/data bits and pulsing EN.
%%%
%%% The HD44780 is used in 4-bit mode: each byte of data/command is
%%% sent as two nibbles (high 4 bits first, then low 4 bits). This
%%% means each character or command requires two I2C writes (plus EN
%%% pulses), but it only needs 4 data lines — leaving P0-P3 free for
%%% control signals.
%%%
%%% Usage:
%%%   I2C = i2c:open([{sda, 8}, {scl, 9}, {peripheral, 0}, {clock_speed_hz, 100000}]),
%%%   {ok, LCD} = lcd1602:init(I2C),
%%%   lcd1602:clear(LCD),
%%%   lcd1602:write_string(LCD, 0, 0, "Hello World!"),
%%%   lcd1602:write_string(LCD, 1, 0, "Line 2").
%%% @end
%%%-------------------------------------------------------------------
-module(lcd1602).

-export([init/1, init/2, clear/1, home/1,
         set_cursor/3, write_string/4, backlight/2]).

-define(LCD_ADDR, 16#27).

%% HD44780 commands
-define(CMD_CLEAR, 16#01).
-define(CMD_HOME, 16#02).
-define(CMD_ENTRY_MODE, 16#06).
-define(CMD_DISPLAY_ON, 16#0C).
-define(CMD_FUNCTION_SET, 16#28).  %% 4-bit, 2 lines, 5x8
-define(CMD_SET_DDRAM, 16#80).

%% PCF8574 control bits
-define(RS, 16#01).   %% P0: Register Select (0=cmd, 1=data)
-define(RW, 16#02).   %% P1: Read/Write (always 0 for write)
-define(EN, 16#04).   %% P2: Enable
-define(BL, 16#08).   %% P3: Backlight

-record(lcd, {
    i2c :: i2c:i2c(),
    addr :: integer(),
    backlight :: integer()
}).

%% @doc Initialise the LCD on an already-open I2C bus at the default
%% address (16#27). The I2C handle is obtained from i2c:open/1 and may
%% be shared with other devices on the same bus.
-spec init(i2c:i2c()) -> {ok, #lcd{}}.
init(I2C) ->
    init(I2C, ?LCD_ADDR).

%% @doc Initialise the LCD on an already-open I2C bus at the given address.
-spec init(i2c:i2c(), integer()) -> {ok, #lcd{}}.
init(I2C, Addr) ->
    LCD = #lcd{i2c = I2C, addr = Addr, backlight = ?BL},
    ok = init_display(LCD),
    {ok, LCD}.

%% @doc Clear the display.
clear(LCD) ->
    send_command(LCD, ?CMD_CLEAR),
    timer:sleep(2).

%% @doc Move cursor to home position.
home(LCD) ->
    send_command(LCD, ?CMD_HOME),
    timer:sleep(2).

%% @doc Set cursor position. Row 0-1, Col 0-15.
set_cursor(LCD, Row, Col) ->
    Offset = case Row of
        0 -> 16#00;
        1 -> 16#40;
        _ -> 16#00
    end,
    send_command(LCD, ?CMD_SET_DDRAM bor (Offset + Col)).

%% @doc Write a string at the given row and column.
write_string(LCD, Row, Col, String) ->
    set_cursor(LCD, Row, Col),
    lists:foreach(fun(Char) -> send_data(LCD, Char) end, String).

%% @doc Control backlight. true = on, false = off.
backlight(LCD = #lcd{}, On) ->
    BL = case On of true -> ?BL; false -> 0 end,
    LCD1 = LCD#lcd{backlight = BL},
    %% Send a no-op to update backlight state
    i2c:write_bytes(LCD1#lcd.i2c, LCD1#lcd.addr, <<BL>>),
    LCD1.



%%%===================================================================
%%% Internal
%%%===================================================================

init_display(LCD) ->
    %% HD44780 initialization sequence for 4-bit mode.
    %%
    %% On power-up the LCD controller's state is unknown — it might
    %% be stuck waiting for the second nibble of a previous byte
    %% (e.g. if the ESP32 reset mid-transfer). The datasheet-
    %% prescribed recovery is to send 0x03 ("Function Set: 8-bit")
    %% three times. This forces the controller into a known 8-bit
    %% state regardless of where it was.
    %%
    %% Then we send 0x02 to switch to 4-bit mode. From here on,
    %% every byte is sent as two 4-bit nibbles (high first, low
    %% second) via send_command/send_data.
    timer:sleep(50),
    write_4bits(LCD, 16#03 bsl 4),
    timer:sleep(5),
    write_4bits(LCD, 16#03 bsl 4),
    timer:sleep(5),
    write_4bits(LCD, 16#03 bsl 4),
    timer:sleep(1),
    %% Switch to 4-bit mode
    write_4bits(LCD, 16#02 bsl 4),
    timer:sleep(1),
    %% Now in 4-bit mode, configure display
    send_command(LCD, ?CMD_FUNCTION_SET),  %% 4-bit, 2 lines, 5x8 font
    send_command(LCD, ?CMD_DISPLAY_ON),    %% display on, cursor off
    send_command(LCD, ?CMD_CLEAR),         %% clear screen
    timer:sleep(2),
    send_command(LCD, ?CMD_ENTRY_MODE),    %% auto-increment cursor
    ok.

%% Send a command byte (RS=0)
send_command(LCD, Cmd) ->
    send_byte(LCD, Cmd, 0).

%% Send a data byte (RS=1)
send_data(LCD, Data) ->
    send_byte(LCD, Data, ?RS).

%% Send a byte in two 4-bit nibbles.
%% The byte is split into high and low nibbles. Each nibble is placed
%% in bits P4-P7 of the I2C byte (the data lines), with control bits
%% (RS, backlight) in P0-P3. Each nibble is latched by pulsing EN.
send_byte(LCD, Byte, Mode) ->
    HighNibble = (Byte band 16#F0) bor Mode bor LCD#lcd.backlight,
    LowNibble = ((Byte bsl 4) band 16#F0) bor Mode bor LCD#lcd.backlight,
    write_4bits(LCD, HighNibble),
    write_4bits(LCD, LowNibble).

%% Write a 4-bit nibble by pulsing the Enable pin.
%% Value already has data in bits 4-7 and control in bits 0-3.
%% EN high → LCD reads the data lines → EN low → LCD latches.
write_4bits(LCD, Value) ->
    Addr = LCD#lcd.addr,
    I2C = LCD#lcd.i2c,
    %% EN high
    i2c:write_bytes(I2C, Addr, <<(Value bor ?EN)>>),
    timer:sleep(1),
    %% EN low (latch)
    i2c:write_bytes(I2C, Addr, <<(Value band (bnot ?EN))>>),
    timer:sleep(1).
