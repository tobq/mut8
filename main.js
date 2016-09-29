window.requestAnimationFrame || (window.requestAnimationFrame = window.webkitRequestAnimationFrame ||
								 window.mozRequestAnimationFrame || window.oRequestAnimationFrame ||
								 window.msRequestAnimationFrame || function(b) {
	window.setTimeout(b, 1E3 / 60);
});
var world, canvas = document.getElementById("canvas"),
	ctx = canvas.getContext("2d"),
	view = {
		scale: 30,
		xoff: 0,
		yoff: 0
	},
	b2Vec2 = Box2D.Common.Math.b2Vec2,
	b2BodyDef = Box2D.Dynamics.b2BodyDef,
	b2Body = Box2D.Dynamics.b2Body,
	b2FixtureDef = Box2D.Dynamics.b2FixtureDef,
	b2Fixture = Box2D.Dynamics.b2Fixture,
	b2World = Box2D.Dynamics.b2World,
	b2MassData = Box2D.Collision.Shapes.b2MassData,
	b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape,
	b2CircleShape = Box2D.Collision.Shapes.b2CircleShape,
	b2DebugDraw = Box2D.Dynamics.b2DebugDraw,
	b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef,
	debugDraw = new b2DebugDraw(),
	legs = [],
	flags = {
		ResetLegs: 0,
		play: 1
	},
	config = {
		PARENTS: 5,
		CHILDREN: 25,
		SPEED: 1,
		RUNTIME: 20,
		SPAWNX: 4,
		SPAWNY: 8,
		MUTATIONRATE: 5,
		TIMESTEP: 1 / 60,
		MAXMOTORSPEED: 10,
		MAXMOTORTORQUE: 100
	},
	parents = [],
	delta = Date.now(),
	elapsed = 0,
	maxDist = config.SPAWNX / 2,
	graph = {
		average:[],
		max:[]
	};

world = new b2World(new b2Vec2(0, 9.81), true);

function mutate(p) { return !~~(Math.random() * (100 / (p || config.MUTATIONRATE))) }
function randpol(vs, gx, gy, settings) {
	var bodyDef = new b2BodyDef(),
		fixDef = new b2FixtureDef(),
		vertices = settings ? settings.vertices : [],
		vs = vs && 2 < vs ? vs : ~~(Math.random() * 7) + 3,
		b;
	fixDef.density = settings && settings.density ? settings.density : 1;
	fixDef.friction = settings && settings.friction ? settings .friction : Math.random();
	fixDef.restitution = settings && settings.restitution ? settings.restitution : Math.random();
	fixDef.filter.groupIndex = -1;
	fixDef.shape = new b2PolygonShape();
	bodyDef.type = b2Body.b2_dynamicBody;
	if (!vertices.length) {
		var angles = [];
		for (var i = vs; i--;) {
			angles.push(Math.random() * 2 * Math.PI);
		}
		angles.sort();
		for (var i = 0; i < vs; i++) {
			vertices.push(new b2Vec2(Math.cos(angles[i]), Math.sin(angles[i])));
		}
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
function randpm(r) { return Math.random() * (r || 1) * (Math.random() < .5 ? 1 : -1); }
function createlegs(bA, bB, settings) {
	var joint_def = new b2RevoluteJointDef();
	joint_def.bodyA = bA && bA.CreateFixture ? bA : randpol(null, config.SPAWNX, config.SPAWNY, settings?settings[0]:null);
	joint_def.bodyB = bB && bB.CreateFixture ? bB : randpol(null, config.SPAWNX, config.SPAWNY, settings?settings[1]:null);
	joint_def.localAnchorA = new b2Vec2(joint_def.bodyA.settings.j11 = settings ? settings[0].j11 || randpm():randpm(), joint_def.bodyA.settings.j12 = settings ? settings[0].j12 || randpm():randpm());
	joint_def.localAnchorB = new b2Vec2(joint_def.bodyA.settings.j21 = settings ? settings[0].j21 || randpm():randpm(), joint_def.bodyA.settings.j22 = settings ? settings[0].j22 || randpm():randpm());
	joint_def.enableMotor = true;
	joint_def.motorSpeed = joint_def.bodyA.settings.motorSpeed = settings ? settings[0].motorSpeed || Math.random() * config.MAXMOTORSPEED : Math.random() * config.MAXMOTORSPEED;
	joint_def.maxMotorTorque = joint_def.bodyA.settings.maxMotorTorque = settings ? settings[0].maxMotorTorque || Math.random() * config.MAXMOTORTORQUE : Math.random() * config.MAXMOTORTORQUE;
	legs.push([joint_def.bodyA, joint_def.bodyB]);
	return world.CreateJoint(joint_def);
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
	if (e.keyCode === 37) config.SPEED = Math.max(0, Math.round(config.SPEED) - 1);
	else if (e.keyCode === 39) config.SPEED = Math.round(config.SPEED) + 1;
	else if (e.keyCode === 40) config.SPEED = config.SPEED < 2 ? 0 : config.SPEED / 2;
	else if (e.keyCode === 38)  config.SPEED = config.SPEED ? config.SPEED * 2 : 1;
	document.getElementById("speed").innerHTML = config.SPEED ? "x" + config.SPEED : "PAUSED";
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
	if (flags.ResetLegs || elapsed > config.RUNTIME) {
		elapsed = 0;
		var i = legs.length, avg=0, ptotal=0;
		while (i--) avg += legs[i][0].settings.fx = (legs[i][0].m_xf.position.x + legs[i][1].m_xf.position.x) / 2;
		graph.average.push(avg/config.CHILDREN);
		legs.sort(function(a, b) { return b[0].settings.fx - a[0].settings.fx });
		graph.max.push(legs[0][0].settings.fx);
		if (legs[0][0].settings.fx > maxDist) maxDist = legs[0][0].settings.fx;
		for (var i = config.PARENTS; i--;) {
			parents[i] = [legs[i][0].settings, legs[i][1].settings];
			ptotal+= legs[i][0].settings.fx;
		}
		while (legs.length) {
			window.Legs = legs.pop();
			world.DestroyBody(Legs[0]);
			world.DestroyBody(Legs[1]);
		}
		for (var z = config.PARENTS; z--;) {
			var p = parents[z];
			createlegs(null, null, p);
			for (var i = 5; i--;) {
				createlegs(null, null, [{
					restitution: mutate()?null:(p[0].restitution + parents[~~(Math.random()*config.PARENTS)][0].restitution) / 2,
					friction: mutate()? null:(p[0].friction + parents[~~(Math.random()*config.PARENTS)][0].friction) / 2,
					density: mutate()? null:(p[0].density + parents[~~(Math.random()*config.PARENTS)][0].density) / 2,
					vertices: mutate(100/config.CHILDREN) ? [] : p[0].vertices,
					motorSpeed: mutate()?null:(p[0].motorSpeed + parents[~~(Math.random()*config.PARENTS)][0].motorSpeed) / 2,
					j11: mutate()?null:(p[0].j11 + parents[~~(Math.random()*config.PARENTS)][0].j11) / 2,
					j12: mutate()?null:(p[0].j12 + parents[~~(Math.random()*config.PARENTS)][0].j12) / 2,
					j21: mutate()?null:(p[0].j21 + parents[~~(Math.random()*config.PARENTS)][0].j21) / 2,
					j22: mutate()?null:(p[0].j22 + parents[~~(Math.random()*config.PARENTS)][0].j22) / 2,
					maxMotorTorque: mutate()?null:(p[0].maxMotorTorque + parents[~~(Math.random()*config.PARENTS)][0].maxMotorTorque) / 2
				}, {
					restitution: mutate()? null:(p[1].restitution + parents[~~(Math.random()*config.PARENTS)][1].restitution) / 2,
					friction: mutate()? null:(p[1].friction + parents[~~(Math.random()*config.PARENTS)] [1].friction) / 2,
					density: mutate()? null:(p[1].density + parents[~~(Math.random()*config.PARENTS)][1].density) / 2,
					vertices: mutate(100/config.CHILDREN) ? [] : p[1].vertices,
				}]);
			}
		}
	}
	var d = (Date.now() - delta) * config.SPEED / 1E3,
		gw = Math.max(graph.average.length,graph.max.length),
		ratio = gw*view.scale > canvas.width ? canvas.width/(gw*view.scale):1;
	elapsed += d;
	while ((d -= config.TIMESTEP) > 0)  world.Step(config.TIMESTEP, 8, 3);
	delta = Date.now() + d;
	ctx.save();
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	ctx.translate(view.xoff, view.yoff);
	world.DrawDebugData();
	ctx.fillStyle = "rgba(200,0,0,0.5)";
	ctx.fillRect(maxDist * view.scale, 11 * view.scale, 4 * view.scale, 4 * view.scale);
	ctx.fillRect(31/8*view.scale, 11 * view.scale, view.scale/4, 4 * view.scale);
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
	ctx.restore();
	world.ClearForces();
	requestAnimationFrame(update);
}
