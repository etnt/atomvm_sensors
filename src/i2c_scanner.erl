%% @doc I2C bus scanner for Raspberry Pi Pico.
%%
%% Scans all valid 7-bit addresses (0x08–0x77) and prints devices that ACK.
%% Uses I2C0 on GP4 (SDA) and GP5 (SCL).
%%
%% Expected addresses for Qwiic sensors:
%%   0x10 or 0x48  VEML6030 (ambient light)
%%   0x58          SGP30    (air quality)
%%   0x76 or 0x77  BME680   (temp/hum/press/gas)

-module(i2c_scanner).
-export([start/0]).

start() ->
    I2C = i2c:open([{sda, 4}, {scl, 5}, {peripheral, 0}, {clock_speed_hz, 100000}]),
    io:format("I2C bus scan on GP4 (SDA), GP5 (SCL)...~n"),
    Found = scan(I2C, 16#08, []),
    case Found of
        [] ->
            io:format("No devices found.~n");
        _ ->
            io:format("Done. Found ~B device(s).~n", [length(Found)])
    end.

scan(_I2C, Addr, Acc) when Addr > 16#77 ->
    lists:reverse(Acc);
scan(I2C, Addr, Acc) ->
    NewAcc =
        case i2c:read_bytes(I2C, Addr, 1) of
            {ok, _} ->
                io:format("  0x~2.16.0B  ~s~n", [Addr, describe(Addr)]),
                [Addr | Acc];
            {error, _} ->
                Acc
        end,
    scan(I2C, Addr + 1, NewAcc).

describe(16#10) -> "VEML6030 (ambient light)";
describe(16#48) -> "VEML6030 (ambient light, alt addr)";
describe(16#58) -> "SGP30 (eCO2/TVOC)";
describe(16#76) -> "BME680 (temp/hum/press/gas)";
describe(16#77) -> "BME680 (temp/hum/press/gas, alt addr)";
describe(_) -> "".
