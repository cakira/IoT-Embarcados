% =========================================================================
% Tipo: MATLAB Analysis
% Gatilho: React App (acionado por novas inserções no Canal Request)
% 
% DESCRIÇÃO:
% Este script calcula a temperatura estimada para uma coordenada alvo 
% (latitude/longitude) aplicando a Equação do Plano sobre os dados dos 
% três sensores ativos mais próximos da requisição.
% 
% CONTEXTO DO SISTEMA:
% Conceitualmente, este script atua como o motor matemático de um serviço 
% solicitado através de um dashboard no Ubidots. Tecnicamente, a interface 
% ocorre exclusivamente com o Node-RED: é o Node-RED quem atualiza os canais 
% 'Geosensors' e 'Request', e assina o canal 'Response' para coletar o 
% resultado e encaminhá-lo ao usuário final no Ubidots.
% =========================================================================

% ==========================================
% CONFIGURATION
% ==========================================
% Channel Geosensors: The JSON Geosensors (data provided by the ESP32)
ch_geosensors = 3272793; 
readKey_geosensors = 'INUJ0K87A4ODP7QC';


% Channel Request: The Temperature Request (the request is made in Ubidots)
ch_requests = 3272803;
readKey_requests = '41T552IF02YIO4QW';


% Channel Response: The Results (filled by this script to be sent do Ubidots)
ch_results = 3272805;
writeKey_results = 'T9G8AWMIJ13BA6P0';

% ==========================================
% READ THE REQUEST (Target Coordinates)
% ==========================================
% Read the latest request from Channel Request
reqData = thingSpeakRead(ch_requests, 'ReadKey', readKey_requests, 'NumPoints', 1);

if isempty(reqData)
    disp('Error: No request data found in Channel Request.');
    return;
end

req_id = reqData(1);
req_x  = reqData(2); % Target Latitude
req_y  = reqData(3); % Target Longitude

% ==========================================
% READ & PARSE THE JSON SENSOR DATA
% ==========================================
% Read Field 1 from Channel Geosensors as a table to extract the raw string
sensorTable = thingSpeakRead(ch_geosensors, 'Fields', 1, 'ReadKey', readKey_geosensors, 'NumPoints', 1, 'OutputFormat', 'table');

if isempty(sensorTable) || ismissing(sensorTable.json_data)
    disp('Error: No sensor data found in Channel Geosensors.');
    return;
end

% Extract the JSON string and decode it
jsonString = sensorTable.json_data{1}; 
try
    data = jsondecode(jsonString);
    sensors = data.sensors;
catch
    disp('Error: Failed to decode JSON string.');
    return;
end

numSensors = length(sensors);
if numSensors < 3
    disp('Error: Need at least 3 sensors to calculate a plane.');
    return;
end

% ==========================================
% FIND THE 3 NEAREST SENSORS
% ==========================================
% When there are more than 3 sensors, the script chose 3 sensors nearest to
% the target latitude and longitude.
% Calculate Euclidean distance for all sensors: d = sqrt((x2-x1)^2 + (y2-y1)^2)
distances = zeros(numSensors, 1);

for i = 1:numSensors
    % Note: MATLAB jsondecode converts JSON arrays into struct arrays
    dist = sqrt((req_x - sensors(i).lat)^2 + (req_y - sensors(i).lon)^2);
    distances(i) = dist;
end

% Sort the distances from smallest to largest
[~, sortedIndices] = sort(distances);

% Grab the indices of the 3 closest sensors
top3 = sortedIndices(1:3);

% Extract their coordinates and temperatures for the math
x = [sensors(top3(1)).lat; sensors(top3(2)).lat; sensors(top3(3)).lat];
y = [sensors(top3(1)).lon; sensors(top3(2)).lon; sensors(top3(3)).lon];
z = [sensors(top3(1)).temp; sensors(top3(2)).temp; sensors(top3(3)).temp];

disp('Using the 3 nearest sensors:');
disp(['Sensor 1 (ID: ', sensors(top3(1)).id, ') Temp: ', num2str(z(1))]);
disp(['Sensor 2 (ID: ', sensors(top3(2)).id, ') Temp: ', num2str(z(2))]);
disp(['Sensor 3 (ID: ', sensors(top3(3)).id, ') Temp: ', num2str(z(3))]);

% ==========================================
% THE GEOMETRY (Plane Equation)
% ==========================================
% Calculate 2 Vectors
v1 = [x(2)-x(1), y(2)-y(1), z(2)-z(1)];
v2 = [x(3)-x(1), y(3)-y(1), z(3)-z(1)];

% Calculate Normal Vector (Cross Product)
normalVec = cross(v1, v2);
a = normalVec(1);
b = normalVec(2);
c = normalVec(3);

if c == 0
    disp('Error: The 3 nearest sensors are collinear (straight line). Cannot calculate plane.');
    return;
end

% Solve for Z (Temperature) at the requested (x, y) target
dx = req_x - x(1);
dy = req_y - y(1);
estimatedTemp = z(1) - ((a * dx) + (b * dy)) / c;

% ==========================================
% WRITE THE RESULTS
% ==========================================
% Write the original Request ID to Field 1 and the Result to Field 2
thingSpeakWrite(ch_results, [req_id, estimatedTemp], 'WriteKey', writeKey_results);
disp(['Success! Wrote Request ID ', num2str(req_id), ' | Estimated Temp: ', num2str(estimatedTemp)]);
