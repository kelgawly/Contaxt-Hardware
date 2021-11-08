DROP TABLE IF EXISTS sensorData;

CREATE TABLE sensorData(
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    forceReading FLOAT,
    creationDateTime DATETIME
);

