%% @doc VEML6030 ambient light sensor driver.
%%
%% Datasheet reference: Vishay VEML6030
%% I2C address: 0x10 (ADDR pin low) or 0x48 (ADDR pin high).
%%
%% Registers (16-bit little-endian data):
%%   0x00  ALS_CONF   - configuration (gain, integration time, power)
%%   0x04  ALS        - ambient light sensor output data (raw counts)
%%   0x05  WHITE      - white channel output data (raw counts)
%%
%% Resolution (lux per count) depends on gain and integration time.
%% Default: gain=1, IT=100ms → resolution = 0.0576 lux/count.
%%
%% The VEML6030 has two photodiodes with different spectral filters:
%%
%% ALS channel — Filtered to approximate human eye response (photopic curve,
%%               peaks ~550nm green). This gives you a lux reading that matches
%%  how bright a scene looks to humans.
%%
%% White channel — Much broader spectral sensitivity (roughly 400–800nm),
%%                 responding to visible + some near-IR. It picks up more of
%%                 the total optical power regardless of wavelength.
%%
%% Practical differences:
%%
%% Under sunlight or incandescent light (broad spectrum), White reads
%% significantly higher than ALS.
%% Under green-ish light (e.g. fluorescent), they'll be closer together.
%% The ratio White/ALS can hint at the light source type — a high ratio
%% suggests IR-rich sources (incandescent, sunlight), a low ratio suggests
%% narrow-band (LED, fluorescent).

-module(veml6030).
-export([init/2, read_lux/1, read_white/1, read_raw/1]).

-record(veml, {i2c, addr, resolution}).

%% Registers
-define(REG_ALS_CONF, 16#00).
-define(REG_ALS_DATA, 16#04).
-define(REG_WHITE_DATA, 16#05).

%% Configuration bits (register 0x00, 16-bit little-endian)
%% Bits [12:11] = ALS_GAIN   00=x1, 01=x2, 10=x(1/8), 11=x(1/4)
%% Bits [9:6]   = ALS_IT     0000=25ms, 0001=50ms, 0010=100ms, 0011=200ms,
%%                            1000=400ms, 1100=800ms
%% Bit  [0]     = ALS_SD     0=power on, 1=shutdown

%% @doc Initialise the VEML6030 with default settings (gain x1, 100ms).
%% Returns an opaque handle for subsequent reads.
-spec init(pid(), non_neg_integer()) -> #veml{}.
init(I2C, Addr) ->
    %% ALS_GAIN=x1 (00), ALS_IT=100ms (0010 in bits 9:6 = 0x0080),
    %% ALS_SD=0 (power on). Config word = 0x0000 for gain x1, 100ms.
    %% Bits: [15:13]=0, [12:11]=00(gain x1), [10]=0, [9:6]=0000(100ms), 
    %%       [5:4]=00(pers 1), [3:2]=00, [1]=0(int off), [0]=0(on)
    Config = <<16#00, 16#00>>,
    ok = i2c:write_bytes(I2C, Addr, ?REG_ALS_CONF, Config),
    timer:sleep(5),
    #veml{i2c = I2C, addr = Addr, resolution = 0.0576}.

%% @doc Read ambient light in lux.
-spec read_lux(#veml{}) -> {ok, float()}.
read_lux(#veml{resolution = Res} = V) ->
    {ok, Raw} = read_raw(V),
    {ok, Raw * Res}.

%% @doc Read white channel in lux (approximate).
-spec read_white(#veml{}) -> {ok, float()}.
read_white(#veml{i2c = I2C, addr = Addr, resolution = Res}) ->
    {ok, <<Low, High>>} = i2c:read_bytes(I2C, Addr, ?REG_WHITE_DATA, 2),
    Raw = (High bsl 8) bor Low,
    {ok, Raw * Res}.

%% @doc Read raw ALS count (16-bit unsigned).
-spec read_raw(#veml{}) -> {ok, non_neg_integer()}.
read_raw(#veml{i2c = I2C, addr = Addr}) ->
    {ok, <<Low, High>>} = i2c:read_bytes(I2C, Addr, ?REG_ALS_DATA, 2),
    {ok, (High bsl 8) bor Low}.
