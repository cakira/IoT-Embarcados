% =========================================================================
% Tipo: MATLAB Visualization
% Exibir no Painel do Canal Geosensors
% 
% DESCRIÇÃO:
% Este script lê o pacote JSON armazenado no canal Geosensors, decodifica 
% os dados de telemetria da rede (ID, latitude, longitude e temperatura) 
% e gera uma tabela visual estruturada, ordenada pelo identificador numérico
% de cada sensor.
% 
% CONTEXTO DO SISTEMA:
% Atua estritamente como uma ferramenta de auditoria de dados embutida 
% no painel do ThingSpeak. O script desempacota o payload consolidado 
% enviado pelo Node-RED para facilitar a leitura humana, não alterando 
% dados nem interferindo no fluxo de mensagens MQTT ou requisições HTTP 
% da arquitetura principal.
% =========================================================================

% ==========================================
% CONFIGURATION
% ==========================================
ch_geosensors = 3272793; 
readKey_geosensors = 'INUJ0K87A4ODP7QC';

% ==========================================
% READ & PARSE JSON SENSOR DATA
% ==========================================
sensorDataRaw = thingSpeakRead(ch_geosensors, 'Fields', 1, 'ReadKey', readKey_geosensors, 'NumPoints', 1, 'OutputFormat', 'table');

if isempty(sensorDataRaw)
    disp('No sensor data found.'); return;
end

% Extract the JSON string using your exact field name
jsonString = sensorDataRaw.json_data{1}; 
data = jsondecode(jsonString);
sensors = data.sensors;

% ==========================================
% PREPARE & SORT THE DATA
% ==========================================
numSensors = length(sensors);
s_ids = zeros(numSensors, 1);

for i = 1:numSensors
    if ischar(sensors(i).id) || isstring(sensors(i).id)
        s_ids(i) = str2double(sensors(i).id);
    else
        s_ids(i) = sensors(i).id;
    end
end

s_lat = [sensors.lat]';
s_lon = [sensors.lon]';
s_temp = [sensors.temp]';

% Combine and sort the data
matlabTable = table(s_ids, s_lat, s_lon, s_temp, 'VariableNames', {'Sensor', 'Latitude', 'Longitude', 'Temp'});
matlabTable = sortrows(matlabTable, 'Sensor');

% ==========================================
% DRAW THE VISUAL TABLE
% ==========================================
f = figure;

% Hide the default axes but keep the title visible
ax = gca;
ax.Visible = 'off'; 
title('Sensor Data', 'FontSize', 14, 'FontWeight', 'bold', 'Visible', 'on');

colNames = {'Sensor', 'Latitude', 'Longitude', 'Temp (ºC)'};

% Draw the table using a simple, fixed normalized position
uitable(f, 'Data', table2cell(matlabTable), ...
           'ColumnName', colNames, ...
           'RowName', [], ... 
           'ColumnWidth', {60, 100, 100, 80}, ...
           'RowStriping', 'on', ...
           'Units', 'Normalized', ...
           'Position', [0.1, 0.2, 0.8, 0.7]);
