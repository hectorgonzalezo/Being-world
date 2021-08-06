(
SynthDef(\gs, {
		|sync =1, dens=20, grainDur=0.1, durRand=1, buf=0,
		rate=1, pos=0, posSpeed=1, posRand=0,
		grainEnv=(-1), pan=0, panHz=0, panRand=0, gate=1, atk=1, dec=0.3, rel=1, crv=(-1), amp=1, out=0, ampLFO=0, lfoFreq= 0.5|

		var sig, env, densCtrl, durCtrl, posCtrl, panCtrl;

		env = Linen.kr(gate,atk,1,rel,2);

		densCtrl = Select.ar(sync, [Dust.ar(dens), Impulse.ar(dens)]); //para seleccionar si la densidad es fija o aleatoria

		durCtrl = grainDur * LFNoise1.kr(100).exprange(1/durRand, durRand);

		posCtrl = Phasor.ar(0, posSpeed * BufRateScale.ir(buf),0, BufSamples.ir(buf)-1);
		posCtrl = posCtrl + LFNoise1.kr(100).bipolar(posRand*SampleRate.ir);
		posCtrl = posCtrl / BufSamples.ir(buf);
		posCtrl = posCtrl + pos;

		panCtrl = pan + LFNoise1.kr(panHz).bipolar(panRand);

		sig = GrainBuf.ar(
			2,
			densCtrl,
			durCtrl,
			buf,
			rate,
			posCtrl,
			2,
			panCtrl,
			grainEnv
		);

		sig = LeakDC.ar(sig * env);

		Out.ar(out, Select.ar(ampLFO,[sig*amp.lag2(0.1),sig *amp.lag(0.1)*LFNoise1.kr(LFNoise0.kr(lfoFreq).range(0.5,2)).range(0.3,1)]));

	}).add;

~dirpath = "11_apr_21".resolveRelative;
~forest = BuffersFolder.readChannel(~dirpath)

)


(
	Pdef(\chords,
		Pbind(
			\instrument, \gs,
			\atk, Pwhite(3,7),
			\rel, Pwhite(3,5),
			\dur, Pdefn(\chordsDur,Pwrand([
				Rest(7),
				Pwhite(10,30,1)+Pkey(\dur)+Pkey(\rel)],
			[0.3,0.7],inf)),
			\legato, Pwhite(0.9,1.1),
			\buf, ~forest[0],
			\sync, 1,
			\pos, Pwhite(0.1,0.3,inf),
			\posSpeed, Pwhite(0.01,0.05,inf),
			\dens, Pwrand([
				(([ 32.0, 34.0, 37.0, 39.0, 41.0 ]
					+Array.rand(5,0,0.05)
					+Pbrown(0,12,2.0,inf)
				).midicps)*
				Pfunc({Array.fill(5,{[1,2,3].wchoose([10,5,2].normalizeSum)})},),

				(([ 32.0, 34.0, 38.0, 40.0, 44.0 ]
					+Array.rand(5,0,0.05)
					+Pbrown(0,12,2.0,inf)
				).midicps)*
				Pfunc({Array.fill(5,{[1,2,3].wchoose([10,5,2].normalizeSum)})})
			],
			[0.75,0.25],
			inf),
			\grainDur, Pwhite(0.1,0.12,inf),
			\ampLFO,1,
			\lfoFreq, Array.rand(5,0.3,1),
			\amp, Pfunc({Array.fill(5,{rrand(5.0,9.5)})})*Pdefn(\chordAmp,1),
			\pan, Pfunc({Array.fill(5,{rrand(-1.0,1.0).round(0.25)})}),
			\panHz, Pfunc({Array.fill(5,{rrand(0.1,0.5)})}),
			\panRand, Pfunc({Array.fill(5,{rrand(0.3,1).round(0.5)})}),
			\stretch, 1
		)
	);
)
Pdef(\chords).play















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
	~randomHarm = [rrand(0.5, 2.5), 1].choose;

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


0.4.round

a = Synth(\gs, [\buf, ~forest[0], \amp, 1])!5

a.set(\pos, 0.1)
a.set(\amp, 1)
a.set(\sync, 0)

(
OSCdef(\wekinator, {|msg, time, addr, recvPort|
	msg.postln;

	a.do{
		|element, i|
		element.set(
		\sync, msg[1].lincurve(0, 1, 0, 1, -3).round.asInteger,
		\dens, msg[2].linexp(0, 1, 2, 200) * (i+1) * ~randomHarm,
		\grainDur, msg[3].linlin(0, 1, 0.5, 3000.reciprocal),
		\durRand, msg[4]+1,
		\rate, msg[5].lincurve(0, 1, 0.5, 2.5, -3),
		\pos, msg[6],
		\posSpeed, msg[7].linlin(0, 1, 0.5, 1.5),
		\pan, msg[8].linlin(0, 1, -0.8, 0.8),
		\panHz, msg[9].linlin(0, 1, 0, 10),
		\panRand, msg[10].round(0.3),
		\amp, msg[11].lincurve(0, 1, 0.5, 5, -4),
		\ampLFO, msg[12],
		\lfoFreq, msg[13].linexp(0, 1, 0.5, 20));
	}
},
'/wek/outputs',
recvPort: 57120
);
)

0.3.round(0.2)


(0..100).collect(_.lincurve(0, 100, 0, 1, -4)).plot