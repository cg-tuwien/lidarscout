import bpy
import bmesh
from mathutils import Vector

from math import floor

#bpy.ops.mesh.primitive_cube_add()
#box = bpy.context.active_object
#box.location = (1, 1, 0)
#box.scale = (0.5, 0.5, 0.5)

def clamp(x, min, max):
	if (x <= min): return min 
	if (x >= max): return max

	return x

class Box:
	min = Vector([0, 0, 0])
	max = Vector([0, 0, 0])

	def toCube(self):
		cubesize = max(
			self.max.x - self.min.x,
			self.max.z - self.min.z,
			self.max.y - self.min.y)

		box = Box()
		box.min = Vector([self.min.x, self.min.y, self.min.z])
		box.max = Vector([
			self.min.x + cubesize,
			self.min.y + cubesize,
			self.min.z + cubesize,
		])

		return box

def getAABB(object):

	boxmin = Vector([ float("inf"),  float("inf"),  float("inf")])
	boxmax = Vector([-float("inf"), -float("inf"), -float("inf")])

	for vertex in object.bound_box: 
		v = object.matrix_world @ Vector(vertex)
		boxmin.x = min(boxmin.x, v.x)
		boxmin.y = min(boxmin.y, v.y)
		boxmin.z = min(boxmin.z, v.z)
		boxmax.x = max(boxmax.x, v.x)
		boxmax.y = max(boxmax.y, v.y)
		boxmax.z = max(boxmax.z, v.z)

	box = Box()
	box.min = boxmin
	box.max = boxmax
		
	return box

def drawBoundingBox(box):
	cz = (box.max.z + box.min.z) / 2
	width = box.max.x - box.min.x
	depth = box.max.y - box.min.y
	height = box.max.z - box.min.z
	boxsize = Vector([width, depth, height])
	cubesize = max(width, depth, height)

	pwidth = cubesize / 500.0
	plength = 0.5 * cubesize

	collection_main = bpy.context.scene.collection
	collection_root = bpy.data.collections.new("Voxelized")
	collection_bounds = bpy.data.collections.new("BoundingBox")

	collection_root.children.link(collection_bounds)
	bpy.context.scene.collection.children.link(collection_root)

	positions = [
		(box.min.x + 0 * cubesize, box.min.y + 0 * cubesize, cz),
		(box.min.x + 1 * cubesize, box.min.y + 0 * cubesize, cz),
		(box.min.x + 0 * cubesize, box.min.y + 1 * cubesize, cz), 
		(box.min.x + 1 * cubesize, box.min.y + 1 * cubesize, cz), 
		((box.min.x + box.max.x) / 2.0, box.min.y + 0 * cubesize, box.min.z + 0 * cubesize), 
		((box.min.x + box.max.x) / 2.0, box.min.y + 1 * cubesize, box.min.z + 0 * cubesize), 
		((box.min.x + box.max.x) / 2.0, box.min.y + 0 * cubesize, box.min.z + 1 * cubesize), 
		((box.min.x + box.max.x) / 2.0, box.min.y + 1 * cubesize, box.min.z + 1 * cubesize), 
		(box.min.x + 0 * cubesize, (box.min.y + box.max.y) / 2.0, box.min.z + 0 * cubesize), 
		(box.min.x + 1 * cubesize, (box.min.y + box.max.y) / 2.0, box.min.z + 0 * cubesize), 
		(box.min.x + 0 * cubesize, (box.min.y + box.max.y) / 2.0, box.min.z + 1 * cubesize), 
		(box.min.x + 1 * cubesize, (box.min.y + box.max.y) / 2.0, box.min.z + 1 * cubesize), 
	]

	scales = [
		(pwidth, pwidth, plength),
		(pwidth, pwidth, plength),
		(pwidth, pwidth, plength),
		(pwidth, pwidth, plength),
		(plength, pwidth, pwidth),
		(plength, pwidth, pwidth),
		(plength, pwidth, pwidth),
		(plength, pwidth, pwidth),
		(pwidth, plength, pwidth),
		(pwidth, plength, pwidth),
		(pwidth, plength, pwidth),
		(pwidth, plength, pwidth),
	]

	for i in range(12): 
		bpy.ops.mesh.primitive_cube_add(
			location=positions[i],
			scale=scales[i])
		object = bpy.context.active_object
		collection_bounds.objects.link(object)
		collection_main.objects.unlink(object)

	bpy.ops.object.select_all(action='DESELECT')


def voxelize(object, grid, gridsize): 

	aabb = getAABB(object).toCube()
	boxsize = aabb.max - aabb.min

	coords = [(object.matrix_world @ v.co) for v in vertices]

	for coord in coords:
		
		fx = gridsize * (coord.x - aabb.min.x) / boxsize.x
		fy = gridsize * (coord.y - aabb.min.y) / boxsize.y
		fz = gridsize * (coord.z - aabb.min.z) / boxsize.z

		ix = clamp(floor(fx), 0, gridsize - 1)
		iy = clamp(floor(fy), 0, gridsize - 1)
		iz = clamp(floor(fz), 0, gridsize - 1)

		voxelIndex = ix + iy * gridsize + iz * gridsize * gridsize

		grid[voxelIndex] = 1

def drawVoxels(object, grid, gridsize):

	aabb = getAABB(object).toCube()
	boxsize = aabb.max - aabb.min
	voxelsize = boxsize.x / gridsize

	collection_voxels = bpy.data.collections.new("voxels")
	collection_main = bpy.context.scene.collection
	collection_main.children.link(collection_voxels)

	mesh = bpy.data.meshes.new("myBeautifulMesh")
	object_mesh = bpy.data.objects.new(mesh.name, mesh)
	collection_voxels.objects.link(object_mesh)
	bpy.context.view_layer.objects.active = object_mesh

	# verts = [
	# 	( 1.0,  1.0,  0.0), 
	# 	( 1.0, -1.0,  0.0),
	# 	(-1.0, -1.0,  0.0),
	# 	(-1.0,  1.0,  0.0),
	# ]
	# edges = []
	# faces = [[0, 1, 2, 3]]

	vertices = []
	edges = []
	faces = []

	numVoxelsAdded = 0
	facesPerVoxel = 1

	for voxelIndex in range(gridsize * gridsize * gridsize):

		value = grid[voxelIndex]

		if(value == 0): continue

		ix = voxelIndex % gridsize
		iy = floor((voxelIndex % (gridsize * gridsize)) / gridsize)
		iz = floor(voxelIndex / (gridsize * gridsize))

		fx = ix / gridsize
		fy = iy / gridsize
		fz = iz / gridsize

		s = 0.02

		# BOTTOM
		vertices.append((
			(ix + 0.0 + s) * voxelsize + aabb.min.x,
			(iy + 0.0 + s) * voxelsize + aabb.min.y,
			(iz + 0.0 + s) * voxelsize + aabb.min.z,
		))

		vertices.append((
			(ix + 1.0 - s) * voxelsize + aabb.min.x,
			(iy + 0.0 + s) * voxelsize + aabb.min.y,
			(iz + 0.0 + s) * voxelsize + aabb.min.z,
		))

		vertices.append((
			(ix + 1.0 - s) * voxelsize + aabb.min.x,
			(iy + 1.0 - s) * voxelsize + aabb.min.y,
			(iz + 0.0 + s) * voxelsize + aabb.min.z,
		))

		vertices.append((
			(ix + 0.0 + s) * voxelsize + aabb.min.x,
			(iy + 1.0 - s) * voxelsize + aabb.min.y,
			(iz + 0.0 + s) * voxelsize + aabb.min.z,
		))

		# TOP
		vertices.append((
			(ix + 0.0 + s) * voxelsize + aabb.min.x,
			(iy + 0.0 + s) * voxelsize + aabb.min.y,
			(iz + 1.0 - s) * voxelsize + aabb.min.z,
		))

		vertices.append((
			(ix + 1.0 - s) * voxelsize + aabb.min.x,
			(iy + 0.0 + s) * voxelsize + aabb.min.y,
			(iz + 1.0 - s) * voxelsize + aabb.min.z,
		))

		vertices.append((
			(ix + 1.0 - s) * voxelsize + aabb.min.x,
			(iy + 1.0 - s) * voxelsize + aabb.min.y,
			(iz + 1.0 - s) * voxelsize + aabb.min.z,
		))

		vertices.append((
			(ix + 0.0 + s) * voxelsize + aabb.min.x,
			(iy + 1.0 - s) * voxelsize + aabb.min.y,
			(iz + 1.0 - s) * voxelsize + aabb.min.z,
		))

		# BOTTOM
		faces.append([
			8 * numVoxelsAdded + 0,
			8 * numVoxelsAdded + 1,
			8 * numVoxelsAdded + 2,
			8 * numVoxelsAdded + 3,
		])

		# TOP
		faces.append([
			8 * numVoxelsAdded + 4,
			8 * numVoxelsAdded + 5,
			8 * numVoxelsAdded + 6,
			8 * numVoxelsAdded + 7,
		])

		# SIDE 1
		faces.append([
			8 * numVoxelsAdded + 0,
			8 * numVoxelsAdded + 1,
			8 * numVoxelsAdded + 5,
			8 * numVoxelsAdded + 4,
		])

		# SIDE 1
		faces.append([
			8 * numVoxelsAdded + 2,
			8 * numVoxelsAdded + 3,
			8 * numVoxelsAdded + 7,
			8 * numVoxelsAdded + 6,
		])

		# SIDE 1
		faces.append([
			8 * numVoxelsAdded + 1,
			8 * numVoxelsAdded + 2,
			8 * numVoxelsAdded + 6,
			8 * numVoxelsAdded + 5,
		])

		faces.append([
			8 * numVoxelsAdded + 3,
			8 * numVoxelsAdded + 0,
			8 * numVoxelsAdded + 4,
			8 * numVoxelsAdded + 7,
		])

		numVoxelsAdded = numVoxelsAdded + 1

		
		# bpy.ops.mesh.primitive_cube_add(
		# 	location = pos,
		# 	scale = (voxelsize / 2.2, voxelsize / 2.2, voxelsize / 2.2))

		# voxelObject = bpy.context.active_object
		# collection_voxels.objects.link(voxelObject)
		# collection_main.objects.unlink(voxelObject)

	mesh.from_pydata(vertices, edges, faces)

	# for voxelIndex in range(gridsize * gridsize * gridsize):

	# 	value = grid[voxelIndex]

	# 	if(value == 0): continue

	# 	ix = voxelIndex % gridsize
	# 	iy = floor((voxelIndex % (gridsize * gridsize)) / gridsize)
	# 	iz = floor(voxelIndex / (gridsize * gridsize))

	# 	fx = ix / gridsize
	# 	fy = iy / gridsize
	# 	fz = iz / gridsize

	# 	pos = (
	# 		(ix + 0.5) * voxelsize + aabb.min.x,
	# 		(iy + 0.5) * voxelsize + aabb.min.y,
	# 		(iz + 0.5) * voxelsize + aabb.min.z,
	# 	)
		
	# 	bpy.ops.mesh.primitive_cube_add(
	# 		location = pos,
	# 		scale = (voxelsize / 2.2, voxelsize / 2.2, voxelsize / 2.2))

	# 	voxelObject = bpy.context.active_object
	# 	collection_voxels.objects.link(voxelObject)
	# 	collection_main.objects.unlink(voxelObject)












print("start")

object = bpy.data.objects['bunny_small']
vertices = object.data.vertices
#print(vertices)
coords = [(object.matrix_world @ v.co) for v in vertices]

if("MyTestCollection" in bpy.data.collections):
	collection = bpy.data.collections['MyTestCollection']
else:
	collection = bpy.data.collections.new("MyTestCollection")
	bpy.context.scene.collection.children.link(collection)

aabb = getAABB(object)
#drawBoundingBox(aabb.toCube())

gridsize = 32
voxels = [0] * (gridsize * gridsize * gridsize)

voxelize(object, voxels, gridsize)

drawVoxels(object, voxels, gridsize)

print("done")



