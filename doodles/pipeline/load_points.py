import bpy
import bmesh
from mathutils import Vector
from pathlib import Path

from math import floor

print("test abc")

# bpy.ops.mesh.primitive_cube_add()
# box = bpy.context.active_object
# box.location = (1, 1, 0)
# box.scale = (0.5, 0.5, 0.5)


vertices = []
colors = []
edges = []
faces = []



path = "D:\dev\workspaces\ipes_viewer\doodles\pipeline\ca_13_220_230_b4_pts_ms.ascii.ply"
f = Path(bpy.path.abspath(path))

if f.exists():

	text = f.read_text()
	lines = text.split("\n")

	vertexStartLine = 0
	for i in range(0, 100):
		if(lines[i] == "end_header"):
			print("found it")
			vertexStartLine = i + 1
	
	print("vertexStartLine", vertexStartLine)

	minX = 100000000000
	minY = 100000000000
	minZ = 100000000000

	for i in range(vertexStartLine, len(lines), 100):
		line = lines[i]
		tokens = line.split(" ")

		x = float(tokens[0])
		y = float(tokens[1])
		z = float(tokens[2])

		minX = min(minX, x)
		minY = min(minY, y)
		minZ = min(minZ, z)


	# my_coll = bpy.data.collections.new("Cubes")
	# bpy.context.scene.collection.children.link(my_coll)

	for i in range(vertexStartLine, len(lines)):
		line = lines[i]
		tokens = line.split(" ")

		x = (float(tokens[0]) - minX) / 500
		y = (float(tokens[1]) - minY) / 500
		z = (float(tokens[2]) - minZ) / 500

		vertices.append((x, y, z))

		r = float(tokens[3]) / 256
		g = float(tokens[4]) / 256
		b = float(tokens[5]) / 256

		colors.append((r, g, b, 1))

		# bpy.ops.mesh.primitive_uv_sphere_add()
		# box = bpy.context.active_object
		# box.location = (x, y, z)
		# box.scale = (0.03, 0.03, 0.03)
		# my_coll.objects.link(box)

		# bpy.context.scene.collection.objects.unlink(box)









new_mesh = bpy.data.meshes.new('new_mesh')
new_mesh.from_pydata(vertices, edges, faces)
new_mesh.update()

if not new_mesh.vertex_colors:
    new_mesh.vertex_colors.new()

color_layer = new_mesh.vertex_colors.active

bm = bmesh.new()
bm.from_mesh(new_mesh)

for i in range(0, len(bm.vertices)):
	color_layer.data[i].color = colors[i]


new_object = bpy.data.objects.new('new_object', new_mesh)

new_collection = bpy.data.collections.new('new_collection')
bpy.context.scene.collection.children.link(new_collection)

new_collection.objects.link(new_object)