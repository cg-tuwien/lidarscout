
// Requires installing following packages via npm:
//
// npm install "@loaders.gl/core"
// npm install "@loaders.gl/images"
// npm install "@loaders.gl/polyfills"

import * as fs from "fs";
import '@loaders.gl/polyfills'
import {ImageLoader, ImageWriter} from '@loaders.gl/images';
import {encode, load} from '@loaders.gl/core';



class Point{
	constructor(){
		this.x = 0;
		this.y = 0;
		this.z = 0;
		this.r = 0;
		this.g = 0;
		this.b = 0;
	}
}

function clamp(value, min, max){
	return Math.min(Math.max(value, min), max);
}

function toBuffer(arrayBuffer) {
	const buffer = Buffer.alloc(arrayBuffer.byteLength);
	const view = new Uint8Array(arrayBuffer);
	for (let i = 0; i < buffer.length; ++i) {
		buffer[i] = view[i];
	}
	return buffer;
}

let path = "D:/dev/workspaces/ipes_viewer/debug/patch_53_67";

let patchCenter = [0, 0, 0];
{
	let infos = fs.readFileSync(`${path}/info.txt`, "utf8");
	let lines = infos.split("\n");
	let tokens = lines[1].replace(/\s+/g, ' ').trim().split(" ");

	patchCenter = [
		parseFloat(tokens[1]),
		parseFloat(tokens[2]),
		parseFloat(tokens[3]),
	];

}

console.log(patchCenter);

let csv = fs.readFileSync(`${path}/samples.csv`, "utf8");
let lines = csv.split("\n");
let points = [];
for(let line of lines){
	let tokens = line.split(",");

	let point = new Point();
	point.x = parseFloat(tokens[0]) - patchCenter[0];
	point.y = parseFloat(tokens[1]) - patchCenter[1];
	point.z = parseFloat(tokens[2]) - patchCenter[2];
	point.r = parseInt(tokens[3]);
	point.g = parseInt(tokens[4]);
	point.b = parseInt(tokens[5]);

	points.push(point);
}


let framebuffer = new Float32Array(96 * 96 * 4);

// single pixel
// for(let point of points){
// 	// from (-480, 480) to [0, 96)
// 	let px = Math.floor(clamp((point.x + 480) / 10, 0, 95));
// 	let py = Math.floor(clamp((point.y + 480) / 10, 0, 95));
	
// 	let pixelID = px + 96 * py;
	
// 	framebuffer[4 * pixelID + 0] = point.r;
// 	framebuffer[4 * pixelID + 1] = point.g;
// 	framebuffer[4 * pixelID + 2] = point.b;
// 	framebuffer[4 * pixelID + 3] = 1;
// }

let size = 10;
for(let point of points){
	// from (-480, 480) to [0, 96)
	let px = Math.floor(clamp((point.x + 480) / 10, 0, 95));
	let py = Math.floor(clamp((point.y + 480) / 10, 0, 95));
	
	for(let ox = -size; ox <= size; ox++)
	for(let oy = -size; oy <= size; oy++)
	{
		let x = px + ox;
		let y = py + oy;
		let pixelID = x + 96 * y;

		if(x < 0 || x >= 96) continue;
		if(y < 0 || y >= 96) continue;

		let d = clamp(1 - Math.sqrt(ox * ox + oy * oy) / size, 0, 1);
		// let w = clamp(Math.pow(d, 1.5), 0, 1);
		let w = 1 - Math.exp(-d / 10.1);

		let h = 100 + point.z / 2;

		framebuffer[4 * pixelID + 0] += h * w;
		framebuffer[4 * pixelID + 1] += h * w;
		framebuffer[4 * pixelID + 2] += h * w;
		framebuffer[4 * pixelID + 3] += w;
		// framebuffer[4 * pixelID + 0] += point.r * w;
		// framebuffer[4 * pixelID + 1] += point.g * w;
		// framebuffer[4 * pixelID + 2] += point.b * w;
		// framebuffer[4 * pixelID + 3] += w;
		// framebuffer[4 * pixelID + 0] = 255 * w;
		// framebuffer[4 * pixelID + 1] = 255 * w;
		// framebuffer[4 * pixelID + 2] = 255 * w;
		// framebuffer[4 * pixelID + 3] = 1;
		// framebuffer[4 * pixelID + 0] = point.r;
		// framebuffer[4 * pixelID + 1] = point.g;
		// framebuffer[4 * pixelID + 2] = point.b;
		// framebuffer[4 * pixelID + 3] = 1;
	}
	
}

// // single pixel, add on top of blended one to display chunk points
// for(let point of points){
// 	// from (-480, 480) to [0, 96)
// 	let px = Math.floor(clamp((point.x + 480) / 10, 0, 95));
// 	let py = Math.floor(clamp((point.y + 480) / 10, 0, 95));
	
// 	let pixelID = px + 96 * py;
	
// 	framebuffer[4 * pixelID + 0] = point.r;
// 	framebuffer[4 * pixelID + 1] = point.g;
// 	framebuffer[4 * pixelID + 2] = point.b;
// 	framebuffer[4 * pixelID + 3] = 1;
// }


let data = new Uint8Array(96 * 96 * 4);

for(let x = 0; x < 96; x++)
for(let y = 0; y < 96; y++)
{
	let pixelID = x + 96 * y;
	let weight = framebuffer[4 * pixelID + 3];
	data[4 * pixelID + 0] = framebuffer[4 * pixelID + 0] / weight;
	data[4 * pixelID + 1] = framebuffer[4 * pixelID + 1] / weight;
	data[4 * pixelID + 2] = framebuffer[4 * pixelID + 2] / weight;
	data[4 * pixelID + 3] = 255;
}

let testImage = {
	shape: [96, 96, 4],
	data: data,
	width: 96,
	height: 96,
	components: 4,
	layers: [1]
};


const arrayBuffer = await encode(testImage, ImageWriter, {image: {mimeType: 'image/png'}});
fs.writeFileSync('out.png', toBuffer(arrayBuffer));

