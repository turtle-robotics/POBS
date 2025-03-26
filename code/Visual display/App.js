const WebSocket = require('ws');
const express = require('express');
const http = require('http');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

let forceData = {};

wss.on('connection', (ws) => {
    console.log('Client connected');
    ws.send(JSON.stringify(forceData)); // Send initial data
    
    ws.on('message', (message) => {
        try {
            forceData = JSON.parse(message);
            wss.clients.forEach(client => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(JSON.stringify(forceData));
                }
            });
        } catch (error) {
            console.error('Invalid JSON:', error);
        }
    });
});

server.listen(8080, () => console.log('WebSocket server running on port 8080'));
