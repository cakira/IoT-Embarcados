% =========================================================================
% Tipo: MATLAB Visualization
% Exibir no Painel do Canal Response
% 
% DESCRIÇÃO:
% Este script gera um mapa geográfico consolidando o estado da rede de 
% sensores e a última solicitação do usuário. Ele extrai as coordenadas e 
% temperaturas do pacote JSON e as plota utilizando uma escala de cores, 
% sobrepondo o ponto alvo da requisição e o resultado matemático obtido.
% 
% CONTEXTO DO SISTEMA:
% Atua estritamente como uma ferramenta visual de auditoria e depuração 
% embutida no painel do ThingSpeak. O script apenas consome os dados dos 
% três canais da arquitetura (Geosensors, Request e Response) para montar 
% o panorama geoespacial do sistema, não alterando dados nem interagindo 
% com o Node-RED ou o Ubidots.
% =========================================================================

% ==========================================
% CONFIGURATION
% ==========================================
ch_geosensors = 3272793; 
readKey_geosensors = 'INUJ0K87A4ODP7QC';

ch_requests = 3272803;
readKey_requests = '41T552IF02YIO4QW';

ch_results = 3272805;
readKey_results = '525LB153ABX54K72';

% ==========================================
% READ JSON SENSOR DATA
% ==========================================
sensorTable = thingSpeakRead(ch_geosensors, 'Fields', 1, 'ReadKey', readKey_geosensors, 'NumPoints', 1, 'OutputFormat', 'table');

if isempty(sensorTable)
    disp('No sensor data found.'); return;
end

% Directly extract the JSON string using your exact field name
jsonString = sensorTable.json_data{1}; 
data = jsondecode(jsonString);
sensors = data.sensors;

% Extract coordinates and temperatures (transposed into columns)
s_lat = [sensors.lat]';
s_lon = [sensors.lon]';
s_temp = [sensors.temp]';

% ==========================================
% READ LATEST REQUEST & RESULT
% ==========================================
reqData = thingSpeakRead(ch_requests, 'ReadKey', readKey_requests, 'NumPoints', 1);
hasRequest = ~isempty(reqData);

if hasRequest
    req_lat = reqData(2);
    req_lon = reqData(3);
end

% Read the final calculated temperature from Channel 3
resultData = thingSpeakRead(ch_results, 'ReadKey', readKey_results, 'NumPoints', 1);
hasResult = ~isempty(resultData);

if hasResult
    req_temp = resultData(2); % Field 2 holds the estimated temperature
end

% ==========================================
% PLOT THE MAP
% ==========================================
figure;

% Plot the sensors
gs1 = geoscatter(s_lat, s_lon, 150, s_temp, 'filled', 'Marker', 'o');
hold on;

% Add text labels next to each sensor marker
for i = 1:length(s_lat)
    % The spaces before the %.1f add a small gap so the text doesn't overlap the dot
    text(s_lat(i), s_lon(i), sprintf('   %.1f °C', s_temp(i)), 'FontSize', 10, 'FontWeight', 'bold');
end

colormap(jet); 
c = colorbar;
c.Label.String = 'Temperature (°C)';

% Plot the requested location (if it exists)
if hasRequest
    gs2 = geoscatter(req_lat, req_lon, 250, 'magenta', 'filled', 'Marker', 'p');
    
    if hasResult
        % Add text label for the requested location
        text(req_lat, req_lon, sprintf('   %.2f °C', req_temp), 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'magenta');
        
        % Put the final temperature right in the title
        title(sprintf('Sensor Network | Requested Location: %.2f °C', req_temp));
    else
        title('Sensor Network & Requested Location');
    end
    legend([gs1, gs2], {'Sensors', 'Requested Location'}, 'Location', 'best');
else
    legend(gs1, {'Sensors'}, 'Location', 'best');
    title('Sensor Network Map');
end

geobasemap('streets'); 
hold off;
