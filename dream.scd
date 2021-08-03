//receives only information from gestures


(
OSCdef(\test, {|msg, time, addr, recvPort|
	var type, id, state, position, direction, speed, progress, radius, angle, degrees, result;
	//detect kind of gesture
	if(msg.size != 7, {type = 'swipe'}, {type = 'circle'});
	//type.postln;


	if(type == 'swipe',
		{~playSwipe.value(msg)},
		{~playCircle.value(msg);}
	);





},
'/leap/gestures',
recvPort: 57130
);


~playSwipe = {
	|msg |
	var id, state, position, direction, speed;
	        id = msg[1];
			state = msg[2];
			position = msg.copyRange(3, 5);
			direction = msg[6];
			speed = msg[7];
	[id, state, position, direction, speed].postln;

};


~lastDegrees = nil;
~playCircle = {
	|msg |
	var id, progress, radius, angle, degrees, result, coef;
	        id = msg[1];
			progress = msg[2];
			radius = msg[3];
			angle = msg[4];
			degrees = msg[5];
			result = msg[6];
	[id, progress, radius, angle, degrees, result].postln;
	if(result == 'clockwise',
		{coef = 0.3},
		{coef = 0.8});

	if(progress == 'STATE_START',
		{
			(angle.linlin(1, 100, 50, 400)).postln;
			{Pluck.ar(WhiteNoise.ar, 1, 2, (angle.linexp(1, 100, 600, 50)).reciprocal, 2, coef)}.play;
			'ya'.postln;
	});
	~lastDegrees = degrees;


};
)

{Pluck.ar(WhiteNoise.ar, 1, 2, 440.reciprocal, 3, 0.6)}.play


