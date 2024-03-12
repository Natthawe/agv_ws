const express = require('express');
const path = require('path');

const livereload = require('livereload');
const connectLiveReload = require('connect-livereload');

// livereload server
const liveReloadServer = livereload.createServer();
liveReloadServer.watch(path.join(__dirname, 'public'));
liveReloadServer.server.once("connection", () => {
    setTimeout(() => {
        liveReloadServer.refresh("/");
    }, 100);
});

// create express app and port
const app = express();
const port = 3000;

// app set
app.set('views', path.join(__dirname, 'src/view'));
app.use(express.static(path.join(__dirname, 'public')))
app.use(connectLiveReload());

// routes for the index.html
// root route for the index.html
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'src/view/index.html'));
});


// start express server on port 3000
app.listen(port, () => console.log(`sever app listening on port ${port}!`));