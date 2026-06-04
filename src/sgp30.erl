%% @doc SGP30 indoor air quality sensor driver (eCO2 and TVOC).
%%
%% The Sensirion SGP30 is a multi-pixel gas sensor measuring:
%%   - eCO2: equivalent CO2 concentration (400–60000 ppm)
%%   - TVOC: total volatile organic compounds (0–60000 ppb)
%%
%% I2C address: 0x58 (fixed, not configurable).
%%
%% == Protocol ==
%%
%% Unlike register-based sensors (BME680, VEML6030), the SGP30 uses a
%% command-based I2C protocol:
%%   1. Write a 16-bit command (MSB first)
%%   2. Wait for measurement to complete
%%   3. Read response: each 16-bit word comes as [MSB, LSB, CRC8]
%%
%% All data transfers include CRC-8 checksums (polynomial 0x31, init 0xFF).
%%
%% == Baseline Algorithm ==
%%
%% The SGP30 has an on-chip baseline compensation algorithm that tracks
%% sensor drift. For it to work correctly:
%%   - Call measure/1 exactly once per second (±10%)
%%   - First 15 seconds after init return eCO2=400, TVOC=0 (warm-up)
%%   - After warm-up, values become meaningful
%%   - For long-term accuracy, save/restore baseline every hour
%%     (not implemented here for simplicity)
%%
%% == Measurement Interpretation ==
%%
%%   - eCO2 400 ppm = fresh outdoor air baseline
%%   - eCO2 >1000 ppm = stuffy room, ventilation needed
%%   - TVOC 0 ppb = clean air
%%   - TVOC >500 ppb = significant VOC presence
%%
%% SGP30 TVOC (ppb):
%%
%% - Processed output from Sensirion's on-chip algorithm
%% - Reports in ppb (parts per billion) — a calibrated, standardized unit
%% - Has an internal baseline compensation algorithm (needs 1 reading/sec to work)
%% - Uses multiple sensing elements ("multi-pixel") to be more selective
%% - Self-calibrates: tracks the cleanest air it's seen over the last 7 days as baseline
%% - The 400 ppm eCO2 / 0 ppb TVOC baseline is actively maintained by the chip
%%
%% Reference: Sensirion SGP30 datasheet (Version 0.93, March 2018)

-module(sgp30).
-export([init/1, init/2, measure/1, get_serial/1, get_baseline/1, set_baseline/3]).

-record(sgp, {
    i2c,    %% I2C bus handle
    addr    %% 7-bit I2C address (always 0x58)
}).

-define(SGP30_ADDR, 16#58).

%% Commands (16-bit, sent MSB first)
-define(CMD_INIT_AIR_QUALITY, 16#2003).
-define(CMD_MEASURE_AIR_QUALITY, 16#2008).
-define(CMD_GET_BASELINE, 16#2015).
-define(CMD_SET_BASELINE, 16#201E).
-define(CMD_GET_SERIAL, 16#3682).

%% @doc Initialise the SGP30 at default address 0x58.
-spec init(pid()) -> {ok, #sgp{}} | {error, term()}.
init(I2C) ->
    init(I2C, ?SGP30_ADDR).

%% @doc Initialise the SGP30 sensor.
%%
%% Steps:
%%   1. Read serial number to verify the sensor is present
%%   2. Send iaq_init command to start the baseline algorithm
%%   3. Wait 10ms for initialisation to complete
%%
%% After init, call measure/1 once per second for proper operation.
-spec init(pid(), non_neg_integer()) -> {ok, #sgp{}} | {error, term()}.
init(I2C, Addr) ->
    Sgp = #sgp{i2c = I2C, addr = Addr},
    %% SGP30 needs time after power-on before accepting commands
    timer:sleep(100),
    case get_serial(Sgp) of
        {ok, _Serial} ->
            %% Start the IAQ algorithm — must be called before measure
            ok = send_command(I2C, Addr, ?CMD_INIT_AIR_QUALITY),
            timer:sleep(10),
            {ok, Sgp};
        {error, _} = Err ->
            Err
    end.

%% @doc Measure eCO2 (ppm) and TVOC (ppb).
%%
%% Must be called once per second for the baseline algorithm to work.
%% Returns {ok, ECO2, TVOC} where:
%%   - ECO2: 400–60000 ppm (400 = baseline/outdoor air)
%%   - TVOC: 0–60000 ppb (0 = clean air)
%%
%% First 15 seconds after init will return {ok, 400, 0}.
-spec measure(#sgp{}) -> {ok, non_neg_integer(), non_neg_integer()} | {error, term()}.
measure(#sgp{i2c = I2C, addr = Addr}) ->
    ok = send_command(I2C, Addr, ?CMD_MEASURE_AIR_QUALITY),
    %% Measurement takes max 12ms per datasheet
    timer:sleep(12),
    case read_words(I2C, Addr, 2) of
        {ok, [ECO2, TVOC]} ->
            {ok, ECO2, TVOC};
        {error, _} = Err ->
            Err
    end.

%% @doc Read the 48-bit serial number (three 16-bit words).
%% Useful for verifying sensor presence on the bus.
-spec get_serial(#sgp{}) -> {ok, [non_neg_integer()]} | {error, term()}.
get_serial(#sgp{i2c = I2C, addr = Addr}) ->
    ok = send_command(I2C, Addr, ?CMD_GET_SERIAL),
    timer:sleep(1),
    read_words(I2C, Addr, 3).

%% @doc Read the current baseline values for eCO2 and TVOC.
%% Save these periodically and restore after power-cycle with set_baseline/3.
-spec get_baseline(#sgp{}) -> {ok, non_neg_integer(), non_neg_integer()} | {error, term()}.
get_baseline(#sgp{i2c = I2C, addr = Addr}) ->
    ok = send_command(I2C, Addr, ?CMD_GET_BASELINE),
    timer:sleep(10),
    case read_words(I2C, Addr, 2) of
        {ok, [ECO2Base, TVOCBase]} ->
            {ok, ECO2Base, TVOCBase};
        {error, _} = Err ->
            Err
    end.

%% @doc Restore previously saved baseline values.
%% Call this within 7 days of the baseline being saved.
-spec set_baseline(#sgp{}, non_neg_integer(), non_neg_integer()) -> ok | {error, term()}.
set_baseline(#sgp{i2c = I2C, addr = Addr}, ECO2Base, TVOCBase) ->
    %% Set baseline expects: command + TVOC word + CRC + eCO2 word + CRC
    CrcTVOC = crc8(<<(TVOCBase bsr 8), (TVOCBase band 16#FF)>>),
    CrcECO2 = crc8(<<(ECO2Base bsr 8), (ECO2Base band 16#FF)>>),
    %% Pack second command byte + payload into data (first cmd byte as register)
    Data = <<(?CMD_SET_BASELINE band 16#FF),
             (TVOCBase bsr 8), (TVOCBase band 16#FF), CrcTVOC,
             (ECO2Base bsr 8), (ECO2Base band 16#FF), CrcECO2>>,
    i2c:write_bytes(I2C, Addr, ?CMD_SET_BASELINE bsr 8, Data).

%% ---------------------------------------------------------------------------
%% Internal: I2C communication
%% ---------------------------------------------------------------------------

%% Send a 16-bit command to the sensor.
%% Split into "register" (MSB) + "data" (LSB) for the write_bytes/4 API.
%% On the wire this sends [START][ADDR+W][MSB][LSB][STOP] as a single transaction.
send_command(I2C, Addr, Cmd) ->
    i2c:write_bytes(I2C, Addr, Cmd bsr 8, <<(Cmd band 16#FF)>>).

%% Read N words from the sensor. Each word is [MSB, LSB, CRC8].
%% Returns {ok, [Word1, Word2, ...]} or {error, crc_mismatch}.
read_words(I2C, Addr, NumWords) ->
    ByteCount = NumWords * 3,
    case i2c:read_bytes(I2C, Addr, ByteCount) of
        {ok, Data} ->
            parse_words(Data, []);
        {error, _} = Err ->
            Err
    end.

%% Parse response bytes into verified 16-bit words.
%% Each group of 3 bytes = [MSB, LSB, CRC8].
parse_words(<<>>, Acc) ->
    {ok, lists:reverse(Acc)};
parse_words(<<MSB, LSB, CRC, Rest/binary>>, Acc) ->
    case crc8(<<MSB, LSB>>) of
        CRC ->
            Word = (MSB bsl 8) bor LSB,
            parse_words(Rest, [Word | Acc]);
        _Bad ->
            {error, crc_mismatch}
    end.

%% ---------------------------------------------------------------------------
%% Internal: CRC-8 (Sensirion variant)
%%
%% Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
%% Initialisation: 0xFF
%% No final XOR.
%%
%% Used to verify data integrity on every I2C read and to generate
%% checksums for write commands (set_baseline).
%% ---------------------------------------------------------------------------

crc8(Bin) ->
    crc8_bytes(Bin, 16#FF).

crc8_bytes(<<>>, CRC) ->
    CRC;
crc8_bytes(<<Byte, Rest/binary>>, CRC) ->
    crc8_bytes(Rest, crc8_bits(CRC bxor Byte, 8)).

crc8_bits(CRC, 0) ->
    CRC;
crc8_bits(CRC, N) when (CRC band 16#80) =/= 0 ->
    crc8_bits(((CRC bsl 1) band 16#FF) bxor 16#31, N - 1);
crc8_bits(CRC, N) ->
    crc8_bits((CRC bsl 1) band 16#FF, N - 1).
