window.requestAnimationFrame || (window.requestAnimationFrame = window.webkitRequestAnimationFrame ||
								 window.mozRequestAnimationFrame || window.oRequestAnimationFrame ||
								 window.msRequestAnimationFrame || function(b) {
	window.setTimeout(b, 1E3 / 60);
});
var world,
	canvas = document.getElementById("canvas"),
	ctx = canvas.getContext("2d"),
	view = {
		scale: 20,
		xoff: 0,
		yoff: 0
	},
	b2Vec2 = Box2D.Common.Math.b2Vec2,
	b2BodyDef = Box2D.Dynamics.b2BodyDef,
	b2Body = Box2D.Dynamics.b2Body,
	b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
	b2Fixture = Box2D.Dynamics.b2Fixture,
	b2World = Box2D.Dynamics.b2World,
	b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
	b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
	b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
	b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
	debugDraw = new b2DebugDraw(),
	legs = [],
	flags = {
		Reset: 0,
		Finished: 0,
		DrawWorld: 1,
		DrawExtras: 1
	},
	config = {
		PARENTS: 5,
		CHILDREN: 50,
		RUNTIME: 30,
		SPAWNX: 4,
		MUTATIONRATE: 10,
		TIMESTEP: 1 / 60,
		STEPPED: 0,
		MAXMOTORSPEED: 20,
		MAXMOTORTORQUE: 100,
		SHAPESIZE: 1
	},
	parents = [],
	misc = {
		maxDist: config.SPAWNX / 2,
		speed: 100,
		generation: 1
	},
	graph = {
		average:[],
		max:[]
	};

world = new b2World(new b2Vec2(0, 9.81), true);

function mutate(p) { return !~~(Math.random() * (100 / (p || config.MUTATIONRATE))) }
function randpm(r) { return Math.random() * (r || 1) * (Math.random() < .5 ? 1 : -1); }
function mergewr(leg,property) { return (leg[property] + parents[~~(Math.random()*config.PARENTS)][property]) / 2 }
function createpol(vs, gx, gy, settings) {
	var bodyDef = new b2BodyDef(),
		fixDef = new b2FixtureDef(),
		vertices = settings ? settings.vertices || [] : [],
		vs = vs && 2 < vs ? vs : ~~(Math.random() * 8) + 2,
		b;
	fixDef.density = settings  ? settings.density || Math.random() : Math.random();
	fixDef.friction = settings ? settings .friction || Math.random() : Math.random();
	fixDef.restitution = settings ? settings.restitution || Math.random() : Math.random();
	fixDef.filter.groupIndex = -1;
	fixDef.shape = new b2PolygonShape();
	bodyDef.type = b2Body.b2_dynamicBody;
	if (!vertices.length) {
		var angles = [];
		for (var i = vs; i--;) angles.push(Math.random() * 2 * Math.PI);
		angles.sort();
		for (var i = 0; i < vs; i++) vertices.push(new b2Vec2(Math.cos(angles[i])*config.SHAPESIZE, Math.sin(angles[i])*config.SHAPESIZE));
	}
	fixDef.shape.SetAsArray(vertices);
	bodyDef.position.x = gx || Math.random() * 10 + 2;
	bodyDef.position.y = gy || 0;
	b = world.CreateBody(bodyDef);
	b.CreateFixture(fixDef);
	b.settings = {
		vertices: vertices,
		density: fixDef.density,
		friction: fixDef.friction,
		restitution: fixDef.restitution
	};
	return b;
}
function createlegs(bA, bB, settings) {
	this.jointDef = new b2RevoluteJointDef();
	this.jointDef.bodyA = bA || createpol(null, config.SPAWNX, (10-config.SHAPESIZE), settings?settings[0]:null);
	this.jointDef.bodyB = bB || createpol(null, config.SPAWNX, (10-config.SHAPESIZE), settings?settings[1]:null);
	this.jointDef.localAnchorA = new b2Vec2(this.jointDef.bodyA.settings.j11 = settings ? settings[0].j11 || randpm(config.SHAPESIZE):randpm(config.SHAPESIZE), this.jointDef.bodyA.settings.j12 = settings ? settings[0].j12 || randpm(config.SHAPESIZE):randpm(config.SHAPESIZE));
	this.jointDef.localAnchorB = new b2Vec2(this.jointDef.bodyA.settings.j21 = settings ? settings[0].j21 || randpm(config.SHAPESIZE):randpm(config.SHAPESIZE), this.jointDef.bodyA.settings.j22 = settings ? settings[0].j22 || randpm(config.SHAPESIZE):randpm(config.SHAPESIZE));
	this.jointDef.enableMotor = true;
	this.jointDef.motorSpeed = this.jointDef.bodyA.settings.motorSpeed = settings ? settings[0].motorSpeed || Math.random() * config.MAXMOTORSPEED : Math.random() * config.MAXMOTORSPEED;
	this.jointDef.maxMotorTorque = this.jointDef.bodyA.settings.maxMotorTorque = settings ? settings[0].maxMotorTorque || Math.random() * config.MAXMOTORTORQUE : Math.random() * config.MAXMOTORTORQUE;
	legs.push([this.jointDef.bodyA, this.jointDef.bodyB]);
	this.joint = world.CreateJoint(this.jointDef);
}
function clearLegs(){
	while (legs.length) {
		window.Legs = legs.pop();
		world.DestroyBody(Legs[0]);
		world.DestroyBody(Legs[1]);
	}
}
document.onmousedown = function(e) {
	view.mx = e.pageX - view.xoff;
	view.my = e.pageY - view.yoff;
	document.onmousemove = function(e) {
		view.xoff = e.pageX - view.mx;
		view.yoff = e.pageY - view.my;
	};
};
document.onmouseup = function() {
	document.onmousemove = null;
};
document.onwheel = function(e) {
	debugDraw.SetDrawScale(view.scale = e.deltaY > 0 ? Math.pow(view.scale, 1 / 1.01) : Math.pow(view.scale, 1.01));
};
document.onkeydown = function(e) {
	if (e.keyCode === 37) misc.speed = Math.max(0, Math.round(misc.speed) - 1);
	else if (e.keyCode === 39) misc.speed = Math.round(misc.speed) + 1;
	else if (e.keyCode === 40) misc.speed = misc.speed < 2 ? 0 : misc.speed / 2;
	else if (e.keyCode === 38)  misc.speed = misc.speed ? misc.speed * 2 : 1;
	else if (e.keyCode === 87)  flags.DrawWorld = !flags.DrawWorld;
	else if (e.keyCode === 81)  flags.DrawExtras = !flags.DrawExtras;
	else if (e.keyCode === 27)  flags.Reset = 1;
	document.getElementById("speed").innerHTML = misc.speed ? "Speed: x"+misc.speed : "PAUSED";
};
window.onresize = function() {
	canvas.width = window.innerWidth || document.documentElement.clientWidth || document.body.clientWidth;
	canvas.height = window.innerHeight || document.documentElement.clientHeight || document.body.clientHeight;
};
window.onresize();

var fixDef = new b2FixtureDef(),
	bodyDef = new b2BodyDef();
fixDef.density = 1;
fixDef.friction = .5;
fixDef.restitution = .2;
bodyDef.type = b2Body.b2_staticBody;
bodyDef.position.x = 450;
bodyDef.position.y = 13;
fixDef.shape = new b2PolygonShape();
fixDef.shape.SetAsBox(500, 2);
world.CreateBody(bodyDef).CreateFixture(fixDef);

for (var i = config.CHILDREN; i--;) createlegs();
debugDraw.SetSprite(document.getElementById("canvas").getContext("2d"));
debugDraw.SetDrawScale(view.scale);
debugDraw.SetFillAlpha(.2);
debugDraw.SetLineThickness(5);
debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
world.SetDebugDraw(debugDraw);
requestAnimationFrame(update);

function update() {
	if (flags.Reset) {
		flags.Reset = 0;
		misc.generation = 0;
		graph = { average:[], max:[] };
		misc.maxDist = config.SPAWNX / 2;
		parents = [];
		clearLegs();
		for (var i = config.CHILDREN; i--;) createlegs();
	}
	if (flags.Finished) {
		document.getElementById("gen").innerHTML = ++misc.generation;
		config.STEPPED = 0;
		flags.Finished = 0;
		var i = legs.length,
			childrenLeft = config.CHILDREN - config.PARENTS,
			total=0;

		while (i--) total += legs[i][0].settings.fx = (legs[i][0].m_xf.position.x + legs[i][1].m_xf.position.x) / 2;
		graph.average.push(total/config.CHILDREN);
		legs.sort(function(a, b) { return b[0].settings.fx - a[0].settings.fx });
		graph.max.push(legs[0][0].settings.fx);
		if (legs[0][0].settings.fx > misc.maxDist) misc.maxDist = legs[0][0].settings.fx;
		for (var i = config.PARENTS; i--;) parents[i] = [legs[i][0].settings, legs[i][1].settings];

		clearLegs();
		for(var c = config.PARENTS; c--;)  createlegs(null, null, parents[c]);

		while (childrenLeft) {
			var childrenLeftOver = childrenLeft;
			for (var z = config.PARENTS; z--;) for (var i = ~~((config.PARENTS-z)*config.CHILDREN / childrenLeftOver); i--;) {
				if (childrenLeft) {
					var p = parents[z];
					createlegs(null, null, [{
						restitution: mutate()?null: mergewr(p[0],"restitution"),
						friction: mutate()? null: mergewr(p[0],"friction"),
						density: mutate()? null: mergewr(p[0],"density"),
						motorSpeed: mutate()?null: mergewr(p[0],"friction"),
						maxMotorTorque: mutate()?null: mergewr(p[0],"maxMotorTorque"),
						j11: mutate()?null: mergewr(p[0],"j11"),
						j12: mutate()?null: mergewr(p[0],"j12"),
						j21: mutate()?null: mergewr(p[0],"j21"),
						j22: mutate()?null: mergewr(p[0],"j22"),
						vertices: mutate()?null: p[0].vertices
					}, {
						restitution: mutate()?null: mergewr(p[1],"restitution"),
						friction: mutate()?null: mergewr(p[1],"friction"),
						density: mutate()?null: mergewr(p[1],"density"),
						vertices: mutate()?null: p[1].vertices
					}]);
					childrenLeft--;
				} else break;
			}
		}
	}
	var gw = Math.max(graph.average.length,graph.max.length)-1,
		ratio = gw*view.scale > canvas.width ? canvas.width/(gw*view.scale):1
	for (var i = misc.speed;i--;) {
		if (config.STEPPED < config.RUNTIME/config.TIMESTEP) {
			config.STEPPED++;
			world.Step(config.TIMESTEP, 8, 3);
		}
		else {
			flags.Finished = 1;
			break;
		}
	}
	ctx.save();
	ctx.translate(view.xoff, view.yoff);
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	if (flags.DrawWorld) world.DrawDebugData();
	if (flags.DrawExtras) {
		ctx.fillStyle = "rgba(200,0,0,0.5)";
		ctx.fillRect(misc.maxDist * view.scale, 11 * view.scale, 4 * view.scale, 4 * view.scale);
		ctx.fillRect(31/8*view.scale, 11 * view.scale, view.scale/4, 4 * view.scale);
		ctx.lineWidth = view.scale/10;
		ctx.beginPath();
		ctx.strokeStyle = "#FFFF00";
		for (var i = graph.average.length; i--;) ctx.lineTo((i*ratio)*view.scale-view.xoff,(graph.average[i]/20+15)*view.scale);
		ctx.stroke();
		ctx.closePath();
		ctx.beginPath();
		ctx.strokeStyle = "#00FFFF";
		for (var i = graph.max.length; i--;) ctx.lineTo((i*ratio)*view.scale-view.xoff,(graph.max[i]/20+15)*view.scale);
		ctx.stroke();
		ctx.closePath();
	}
	ctx.restore();
	requestAnimationFrame(update);
}
