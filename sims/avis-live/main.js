// Avis, the live page: the bird flying on the flight model in real time over the Bow Valley, or playing back a
// flight log; one button per subsystem. Data files (env/, data/) are served beside the page; see film/serve.mjs.
import { runLive } from '../avis-shared/live.js';

document.title = 'Avis · simulated flight, real parts';
const meta = document.createElement('meta');
meta.name = 'theme-color';
meta.content = '#0c1015';
document.head.appendChild(meta);
const icon = document.createElement('link');
icon.rel = 'icon';
icon.href = 'data:image/svg+xml,%3Csvg xmlns=%22http://www.w3.org/2000/svg%22 viewBox=%220 0 16 16%22%3E%3Ccircle cx=%228%22 cy=%228%22 r=%225%22 fill=%22%23ffb547%22/%3E%3C/svg%3E';
document.head.appendChild(icon);
runLive();
