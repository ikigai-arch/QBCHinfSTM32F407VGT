classdef PILSerialController < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
    % Blok PIL lockstep: kirim [vref, vo_meas] ke STM32 lewat USB CDC,
    % terima duty + jumlah siklus CPU. Pakai lewat blok "MATLAB System".

    properties (Nontunable)
        PortName = "COM5"      % cek nomor COM terbaru (serialportlist)
        BaudRate = 115200      % diabaikan oleh USB CDC
        Ts       = 1e-4        % harus sama dengan CTRL_TS
    end

    properties (Access = private)
        sp
    end

    methods (Access = protected)
        function setupImpl(obj)
            delete(serialportfind);                       % bebaskan port lama
            obj.sp = serialport(obj.PortName, obj.BaudRate, "Timeout", 3);
            setDTR(obj.sp, true);                         % wajib untuk USB CDC STM32duino
            setRTS(obj.sp, true);
            pause(1); flush(obj.sp);
            % reset state kontroler di STM32 (CMD = 1)
            write(obj.sp, [uint8([170 85 1]) zeros(1,8,'uint8')], "uint8");
            read(obj.sp, 10, "uint8");
        end

        function [duty, cycles] = stepImpl(obj, vref, vo)
            pkt = [uint8([170 85 0]), ...
                   typecast(single(vref), 'uint8'), ...
                   typecast(single(vo),   'uint8')];
            write(obj.sp, pkt, "uint8");
            r = read(obj.sp, 10, "uint8");
            duty   = double(typecast(uint8(r(3:6)),  'single'));
            cycles = double(typecast(uint8(r(7:10)), 'uint32'));
        end

        function releaseImpl(obj)
            obj.sp = [];                                  % menutup port
        end

        function st = getSampleTimeImpl(obj)
            st = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function n = getNumInputsImpl(~),  n = 2; end
        function n = getNumOutputsImpl(~), n = 2; end
        function [a,b] = getOutputSizeImpl(~),     a = [1 1]; b = [1 1]; end
        function [a,b] = getOutputDataTypeImpl(~), a = 'double'; b = 'double'; end
        function [a,b] = isOutputComplexImpl(~),   a = false; b = false; end
        function [a,b] = isOutputFixedSizeImpl(~), a = true; b = true; end
    end
end
