%% @doc BME680 environmental sensor driver (temperature, humidity, pressure, gas).
%%
%% The Bosch BME680 is a 4-in-1 sensor measuring:
%%   - Temperature (±1°C accuracy)
%%   - Barometric pressure (±1 hPa, ~300–1100 hPa range)
%%   - Relative humidity (±3% RH)
%%   - Gas resistance (VOC indicator via heated metal-oxide plate)
%%
%% I2C address: 0x76 (SDO pin low) or 0x77 (SDO pin high).
%%
%% == Operating Mode ==
%%
%% This driver uses "forced mode": the sensor sleeps between measurements.
%% Each call to read/1 or read_gas/1 wakes it, takes one measurement, then
%% it returns to sleep (~0.15 µA idle current).
%%
%% == Raw → Physical Conversion ==
%%
%% The sensor outputs raw 20-bit (T, P) or 16-bit (H) ADC values.
%% Each chip has unique factory-programmed calibration coefficients stored
%% in NVM registers. We read these once at init and apply Bosch's
%% compensation formulas (floating-point variant) to convert raw counts
%% to physical units.
%%
%% == Gas Measurement ==
%%
%% The BME680's gas sensor is a tiny metal-oxide (MOX) hot-plate.
%% When heated to ~300–350°C, the oxide layer adsorbs volatile organic
%% compounds (VOCs) from the air, which lowers its electrical resistance.
%% Higher resistance = cleaner air. The heater needs several cycles to
%% stabilize — initial readings are unreliable.
%%
%% What the number means:
%%
%% Higher resistance (e.g. 50–500 kΩ) → cleaner air, fewer VOCs
%% Lower resistance (e.g. 10–50 kΩ) → more VOCs present
%% What it detects (not selectively — it responds to a mix):
%%
%% Breath (CO₂ + moisture + organic compounds)
%% Cooking fumes, alcohol vapour
%% Off-gassing from paint, furniture, plastics
%% Solvents, cleaning products
%%
%% What it does NOT give you:
%%
%% It's not a CO₂ sensor (though it correlates loosely with CO₂ in occupied rooms)
%% It can't identify which gas — just total reducing-gas load
%% The raw Ohm value isn't directly an "air quality index" — Bosch's
%% proprietary BSEC library computes an IAQ score from it, but that's
%% closed-source and not available on AtomVM
%%
%% == AtomVM Register Limit ==
%%
%% AtomVM on RP2040 supports max 16 x-registers per stack frame.
%% To avoid exceeding this, calibration data is split into sub-tuples
%% (cal_t, cal_p, cal_h, cal_g) and complex calculations are broken
%% into smaller helper functions.
%%
%% Reference: Bosch BME680 datasheet (BST-BME680-DS001)

-module(bme680).
-export([init/2, read/1, read_gas/1]).

-record(bme, {
    i2c,             %% I2C bus handle (from i2c:open/1)
    addr,            %% 7-bit I2C address (0x76 or 0x77)
    cal_t,           %% Temperature coefficients: {T1, T2, T3}
    cal_p,           %% Pressure coefficients: {P1..P10}
    cal_h,           %% Humidity coefficients: {H1..H7}
    cal_g            %% Gas/heater coefficients: {G1, G2, G3, RhRange, RhVal, RswErr}
}).

%% ---------------------------------------------------------------------------
%% Register addresses (from BME680 datasheet Table 20)
%% ---------------------------------------------------------------------------
-define(REG_CHIP_ID, 16#D0).      %% Chip identification (should read 0x61)
-define(REG_CTRL_HUM, 16#72).     %% Humidity oversampling control
-define(REG_CTRL_MEAS, 16#74).    %% Temp/pressure oversampling + trigger mode
-define(REG_CTRL_GAS_1, 16#71).   %% Gas measurement enable + heater profile
-define(REG_GAS_WAIT_0, 16#64).   %% Gas heater wait time (profile 0)
-define(REG_RES_HEAT_0, 16#5A).   %% Gas heater resistance target (profile 0)
-define(REG_MEAS_STATUS, 16#1D).  %% Measurement status (bit 7 = measuring)
-define(REG_PRESS_MSB, 16#1F).    %% Start of T/P/H data block (8 bytes)
-define(REG_GAS_R_MSB, 16#2A).    %% Gas resistance data (2 bytes)

-define(CHIP_ID_BME680, 16#61).   %% Expected chip ID value

%% @doc Initialise the BME680 sensor.
%%
%% Steps:
%%   1. Verify chip ID (must be 0x61) to confirm a BME680 is present
%%   2. Read factory calibration coefficients from NVM registers
%%   3. Set humidity oversampling to 1x (written before ctrl_meas per datasheet)
%%
%% Returns {ok, Handle} on success or {error, Reason} on failure.
-spec init(pid(), non_neg_integer()) -> {ok, #bme{}} | {error, term()}.
init(I2C, Addr) ->
    case i2c:read_bytes(I2C, Addr, ?REG_CHIP_ID, 1) of
        {ok, <<?CHIP_ID_BME680>>} ->
            Bme = read_calibration(I2C, Addr),
            %% Humidity oversampling must be set in a separate write
            %% BEFORE writing ctrl_meas (datasheet section 3.3)
            ok = i2c:write_bytes(I2C, Addr, ?REG_CTRL_HUM, <<16#01>>),
            {ok, Bme};
        {ok, <<Other>>} ->
            {error, {unexpected_chip_id, Other}};
        {error, _} = Err ->
            Err
    end.

%% @doc Read temperature (°C), pressure (hPa), and humidity (%RH).
%%
%% Triggers a single forced-mode measurement (no gas heater).
%% ctrl_meas register 0x55 = osrs_t=2x (bits 7:5=010), osrs_p=16x (bits 4:2=101),
%% mode=forced (bits 1:0=01).
-spec read(#bme{}) -> {ok, float(), float(), float()}.
read(#bme{i2c = I2C, addr = Addr} = B) ->
    %% Write ctrl_meas to trigger forced mode measurement
    ok = i2c:write_bytes(I2C, Addr, ?REG_CTRL_MEAS, <<16#55>>),
    wait_measurement(I2C, Addr, 50),
    %% Read 8 bytes of raw ADC data and apply compensation
    {AdcT, AdcP, AdcH} = read_tph_raw(I2C, Addr),
    {TempC, TFine} = calc_temperature(B, AdcT),
    %% TFine is an intermediate value needed by pressure and humidity formulas
    PressHpa = calc_pressure(B, AdcP, TFine),
    HumRh = calc_humidity(B, AdcH, TempC),
    {ok, TempC, PressHpa, HumRh}.

%% @doc Read temperature, pressure, humidity, and gas resistance (Ohms).
%%
%% Same as read/1 but also heats the gas sensor hot-plate to 320°C
%% and measures the MOX layer resistance. The measurement takes longer
%% (~200ms) because of the heater wait time.
%%
%% Gas resistance interpretation:
%%   - Higher resistance (kΩ range) = cleaner air
%%   - Lower resistance = more VOCs present
%%   - First few readings are unreliable (heater stabilising)
-spec read_gas(#bme{}) -> {ok, float(), float(), float(), float()}.
read_gas(#bme{i2c = I2C, addr = Addr} = B) ->
    %% Calculate heater resistance register value for 320°C target
    HeaterRes = calc_heater_resistance(B, 320.0),
    ok = i2c:write_bytes(I2C, Addr, ?REG_RES_HEAT_0, <<HeaterRes>>),
    %% Set heater wait time: 0x59 = 100ms base × 1 multiplier
    %% (bits 7:6 = multiplication factor, bits 5:0 = time step)
    ok = i2c:write_bytes(I2C, Addr, ?REG_GAS_WAIT_0, <<16#59>>),
    %% Enable gas measurement, use heater profile 0
    ok = i2c:write_bytes(I2C, Addr, ?REG_CTRL_GAS_1, <<16#10>>),
    %% Trigger forced mode (same oversampling as read/1)
    ok = i2c:write_bytes(I2C, Addr, ?REG_CTRL_MEAS, <<16#55>>),
    wait_measurement(I2C, Addr, 200),
    %% Read T/P/H and gas ADC values separately
    {AdcT, AdcP, AdcH} = read_tph_raw(I2C, Addr),
    {AdcGas, GasRange} = read_gas_raw(I2C, Addr),
    %% Apply compensation formulas
    {TempC, TFine} = calc_temperature(B, AdcT),
    PressHpa = calc_pressure(B, AdcP, TFine),
    HumRh = calc_humidity(B, AdcH, TempC),
    GasOhms = calc_gas_resistance(B, AdcGas, GasRange),
    {ok, TempC, PressHpa, HumRh, GasOhms}.

%% ---------------------------------------------------------------------------
%% Internal: raw ADC reads
%%
%% Data registers are laid out sequentially starting at 0x1F:
%%   0x1F  press_msb    [19:12]
%%   0x20  press_lsb    [11:4]
%%   0x21  press_xlsb   [3:0] in bits 7:4
%%   0x22  temp_msb     [19:12]
%%   0x23  temp_lsb     [11:4]
%%   0x24  temp_xlsb    [3:0] in bits 7:4
%%   0x25  hum_msb      [15:8]
%%   0x26  hum_lsb      [7:0]
%%
%% Temperature and pressure are 20-bit unsigned values.
%% Humidity is 16-bit unsigned.
%% ---------------------------------------------------------------------------

read_tph_raw(I2C, Addr) ->
    %% Burst-read all 8 bytes in one I2C transaction for efficiency
    {ok, <<P2, P1, P0, T2, T1, T0, H1, H0>>} =
        i2c:read_bytes(I2C, Addr, ?REG_PRESS_MSB, 8),
    %% Assemble 20-bit values from MSB:LSB:XLSB (top 4 bits of XLSB)
    AdcT = (T2 bsl 12) bor (T1 bsl 4) bor (T0 bsr 4),
    AdcP = (P2 bsl 12) bor (P1 bsl 4) bor (P0 bsr 4),
    AdcH = (H1 bsl 8) bor H0,
    {AdcT, AdcP, AdcH}.

read_gas_raw(I2C, Addr) ->
    %% Gas data: 0x2A = gas_r[9:2], 0x2B = gas_r[1:0] | gas_valid | heat_stab | gas_range[3:0]
    {ok, <<GasMsb, GasLsb>>} = i2c:read_bytes(I2C, Addr, ?REG_GAS_R_MSB, 2),
    AdcGas = (GasMsb bsl 2) bor (GasLsb bsr 6),   %% 10-bit ADC value
    GasRange = GasLsb band 16#0F,                   %% Selects lookup table entry
    {AdcGas, GasRange}.

%% ---------------------------------------------------------------------------
%% Internal: wait for measurement to complete
%%
%% The BME680 sets bit 7 of register 0x1D while a measurement is in progress.
%% We poll every 10ms until the bit clears or a timeout is reached.
%% Typical measurement times:
%%   T+P+H only: ~30ms (depends on oversampling)
%%   T+P+H+Gas:  ~180ms (includes heater wait time)
%% ---------------------------------------------------------------------------

wait_measurement(I2C, Addr, MaxMs) ->
    timer:sleep(10),
    wait_loop(I2C, Addr, MaxMs, 10).

wait_loop(_I2C, _Addr, MaxMs, Elapsed) when Elapsed >= MaxMs ->
    ok;
wait_loop(I2C, Addr, MaxMs, Elapsed) ->
    case i2c:read_bytes(I2C, Addr, ?REG_MEAS_STATUS, 1) of
        {ok, <<Status>>} when (Status band 16#80) =:= 0 ->
            ok;  %% Bit 7 clear → measurement complete, data ready
        _ ->
            timer:sleep(10),
            wait_loop(I2C, Addr, MaxMs, Elapsed + 10)
    end.

%% ---------------------------------------------------------------------------
%% Internal: compensation calculations
%%
%% These formulas are transcribed from the Bosch BME680 driver API
%% (bme680_calc.c, floating-point variant). Each takes raw ADC counts
%% and calibration coefficients, returning physical units.
%%
%% Temperature produces an intermediate value "TFine" which encodes
%% the temperature in a form that the pressure and humidity formulas
%% need for their own temperature-dependent corrections.
%% ---------------------------------------------------------------------------

%% Temperature compensation: raw 20-bit ADC → degrees Celsius.
%% Also returns TFine for use by pressure and humidity calculations.
calc_temperature(#bme{cal_t = {T1, T2, T3}}, AdcT) ->
    Var1 = (AdcT / 16384.0 - T1 / 1024.0) * T2,
    Var2 = ((AdcT / 131072.0 - T1 / 8192.0) *
            (AdcT / 131072.0 - T1 / 8192.0)) * T3 * 16.0,
    TFine = Var1 + Var2,
    {TFine / 5120.0, TFine}.

%% Pressure compensation: raw 20-bit ADC → hectopascals (hPa).
%% Split into two steps to keep register usage under 16.
calc_pressure(#bme{cal_p = CalP}, AdcP, TFine) ->
    calc_press_step1(CalP, AdcP, TFine).

calc_press_step1({P1, P2, P3, P4, P5, P6, _P7, _P8, _P9, _P10} = CalP, AdcP, TFine) ->
    Var1 = TFine / 2.0 - 64000.0,
    Var2 = Var1 * Var1 * P6 / 131072.0,
    Var2b = Var2 + Var1 * P5 * 2.0,
    Var2c = Var2b / 4.0 + P4 * 65536.0,
    Var1b = (P3 * Var1 * Var1 / 16384.0 + P2 * Var1) / 524288.0,
    Var1c = (1.0 + Var1b / 32768.0) * P1,
    Press = (1048576.0 - AdcP - Var2c / 4096.0) * 6250.0 / Var1c,
    calc_press_step2(CalP, Press).

calc_press_step2({_P1, _P2, _P3, _P4, _P5, _P6, P7, P8, P9, P10}, Press) ->
    Var1 = P9 * Press * Press / 2147483648.0,
    Var2 = Press * P8 / 32768.0,
    Var3 = (Press / 256.0) * (Press / 256.0) * (Press / 256.0) * P10 / 131072.0,
    (Press + (Var1 + Var2 + Var3 + P7 * 128.0) / 16.0) / 100.0.

%% Humidity compensation: raw 16-bit ADC → %RH (clamped 0–100).
%% Temperature-dependent: uses compensated TempC for correction.
calc_humidity(#bme{cal_h = {H1, H2, H3, H4, H5, H6, H7}}, AdcH, TempC) ->
    Var1 = AdcH - (H1 * 16.0 + H3 / 2.0 * TempC),
    Var2 = Var1 * calc_hum_factor(H2, H4, H5, TempC),
    Var3 = H6 / 16384.0,
    Var4 = H7 / 2097152.0,
    Hum = Var2 + (Var3 + Var4 * TempC) * Var2 * Var2,
    clamp(Hum, 0.0, 100.0).

calc_hum_factor(H2, H4, H5, TempC) ->
    H2 / 262144.0 * (1.0 + H4 / 16384.0 * TempC + H5 / 1048576.0 * TempC * TempC).

%% Gas resistance compensation: 10-bit ADC + range → Ohms.
%% Uses two lookup tables (K1, K2) indexed by the gas_range field
%% to scale the raw ADC value into resistance.
calc_gas_resistance(#bme{cal_g = {_G1, _G2, _G3, _RhRange, _RhVal, RangeSwErr}},
                    AdcGas, GasRange) ->
    K1 = lookup_k1(GasRange),
    K2 = lookup_k2(GasRange),
    Var1 = (1340.0 + 5.0 * RangeSwErr) * K1,
    Var1 * K2 / (AdcGas - 512.0 + Var1).

%% Calculate the register value to set the heater to a target temperature.
%% Uses gas calibration coefficients and assumes ~25°C ambient.
calc_heater_resistance(#bme{cal_g = {G1, G2, G3, RhRange, RhVal, _RswErr}}, TargetTemp) ->
    Var1 = G1 / 16.0 + 49.0,
    Var2 = G2 / 32768.0 * 0.0005 + 0.00235,
    Var3 = G3 / 1024.0,
    Var4 = Var1 * (1.0 + Var2 * TargetTemp),
    Var5 = Var4 + Var3 * 25.0,
    Res = 3.4 * (Var5 * (4.0 / (4.0 + RhRange)) *
                 (1.0 / (1.0 + RhVal * 0.002)) - 25.0),
    round(Res).

%% ---------------------------------------------------------------------------
%% Internal: read calibration data
%%
%% Factory calibration is stored in two non-contiguous NVM regions:
%%   Block 1: registers 0x89–0xA1 (25 bytes) — T2, T3, P1–P10
%%   Block 2: registers 0xE1–0xF0 (16 bytes) — H1–H7, T1, G1–G3
%%   Plus individual registers for heater calibration (0x00, 0x02, 0x04)
%%
%% All multi-byte values are little-endian. Signed values use two's
%% complement. We convert everything to floats (*1.0) at parse time
%% to avoid integer/float conversion overhead during measurements.
%% ---------------------------------------------------------------------------

read_calibration(I2C, Addr) ->
    {ok, Cal1} = i2c:read_bytes(I2C, Addr, 16#89, 25),
    {ok, Cal2} = i2c:read_bytes(I2C, Addr, 16#E1, 16),
    CalT = parse_cal_t(Cal1, Cal2),
    CalP = parse_cal_p(Cal1),
    CalH = parse_cal_h(Cal2),
    CalG = parse_cal_g(I2C, Addr, Cal2),
    #bme{i2c = I2C, addr = Addr, cal_t = CalT, cal_p = CalP,
         cal_h = CalH, cal_g = CalG}.

parse_cal_t(Cal1, Cal2) ->
    T1 = uint16_le(Cal2, 8) * 1.0,
    T2 = sint16_le(Cal1, 1) * 1.0,
    T3 = to_signed8(binary:at(Cal1, 3)) * 1.0,
    {T1, T2, T3}.

parse_cal_p(Cal1) ->
    P1 = uint16_le(Cal1, 5) * 1.0,
    P2 = sint16_le(Cal1, 7) * 1.0,
    P3 = to_signed8(binary:at(Cal1, 9)) * 1.0,
    P4 = sint16_le(Cal1, 11) * 1.0,
    P5 = sint16_le(Cal1, 13) * 1.0,
    P6 = to_signed8(binary:at(Cal1, 16)) * 1.0,
    P7 = to_signed8(binary:at(Cal1, 15)) * 1.0,
    P8 = sint16_le(Cal1, 19) * 1.0,
    P9 = sint16_le(Cal1, 21) * 1.0,
    P10 = binary:at(Cal1, 23) * 1.0,
    {P1, P2, P3, P4, P5, P6, P7, P8, P9, P10}.

parse_cal_h(Cal2) ->
    E1 = binary:at(Cal2, 0),
    E2 = binary:at(Cal2, 1),
    E3 = binary:at(Cal2, 2),
    H2 = ((E1 bsl 4) bor (E2 bsr 4)) * 1.0,
    H1 = ((E3 bsl 4) bor (E2 band 16#0F)) * 1.0,
    H3 = to_signed8(binary:at(Cal2, 3)) * 1.0,
    H4 = to_signed8(binary:at(Cal2, 4)) * 1.0,
    H5 = to_signed8(binary:at(Cal2, 5)) * 1.0,
    H6 = binary:at(Cal2, 6) * 1.0,
    H7 = to_signed8(binary:at(Cal2, 7)) * 1.0,
    {H1, H2, H3, H4, H5, H6, H7}.

parse_cal_g(I2C, Addr, Cal2) ->
    {ok, <<RhRange0>>} = i2c:read_bytes(I2C, Addr, 16#02, 1),
    {ok, <<RhVal0:8/signed>>} = i2c:read_bytes(I2C, Addr, 16#00, 1),
    {ok, <<RswErr0:8/signed>>} = i2c:read_bytes(I2C, Addr, 16#04, 1),
    G1 = to_signed8(binary:at(Cal2, 12)) * 1.0,
    G2 = sint16_le(Cal2, 10) * 1.0,
    G3 = to_signed8(binary:at(Cal2, 13)) * 1.0,
    RhRange = ((RhRange0 bsr 4) band 16#03) * 1.0,
    RhVal = RhVal0 * 1.0,
    RswErr = (RswErr0 bsr 4) * 1.0,
    {G1, G2, G3, RhRange, RhVal, RswErr}.

%% ---------------------------------------------------------------------------
%% Internal: binary helpers
%%
%% BME680 stores calibration as little-endian integers (LSB first).
%% Signed values use standard two's complement representation.
%% ---------------------------------------------------------------------------

%% Read unsigned 16-bit little-endian value at byte offset in binary.
uint16_le(Bin, Offset) ->
    <<_:Offset/binary, Value:16/little-unsigned, _/binary>> = Bin,
    Value.

%% Read signed 16-bit little-endian value at byte offset.
sint16_le(Bin, Offset) ->
    <<_:Offset/binary, Value:16/little-signed, _/binary>> = Bin,
    Value.

to_signed8(V) when V >= 16#80 -> V - 16#100;
to_signed8(V) -> V.

clamp(V, Min, _Max) when V < Min -> Min;
clamp(V, _Min, Max) when V > Max -> Max;
clamp(V, _Min, _Max) -> V.

%% ---------------------------------------------------------------------------
%% Internal: gas resistance lookup tables (from Bosch API)
%%
%% K1: correction factors for manufacturing variation per range.
%% K2: base resistance scaling values (roughly 2^(15-range) * 244).
%% These are empirically determined constants from Bosch's characterisation.
%% ---------------------------------------------------------------------------

lookup_k1(0)  -> 1.0;
lookup_k1(1)  -> 1.0;
lookup_k1(2)  -> 1.0;
lookup_k1(3)  -> 1.0;
lookup_k1(4)  -> 1.0;
lookup_k1(5)  -> 0.99;
lookup_k1(6)  -> 1.0;
lookup_k1(7)  -> 0.992;
lookup_k1(8)  -> 1.0;
lookup_k1(9)  -> 1.0;
lookup_k1(10) -> 0.998;
lookup_k1(11) -> 0.995;
lookup_k1(12) -> 1.0;
lookup_k1(13) -> 0.99;
lookup_k1(14) -> 1.0;
lookup_k1(15) -> 1.0.

lookup_k2(0)  -> 8000000.0;
lookup_k2(1)  -> 4000000.0;
lookup_k2(2)  -> 2000000.0;
lookup_k2(3)  -> 1000000.0;
lookup_k2(4)  -> 499500.4;
lookup_k2(5)  -> 248669.3;
lookup_k2(6)  -> 125000.0;
lookup_k2(7)  -> 63004.03;
lookup_k2(8)  -> 31281.28;
lookup_k2(9)  -> 15625.0;
lookup_k2(10) -> 7812.5;
lookup_k2(11) -> 3906.25;
lookup_k2(12) -> 1953.125;
lookup_k2(13) -> 976.5625;
lookup_k2(14) -> 488.28125;
lookup_k2(15) -> 244.140625.
