(
s.waitForBoot({
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



	SynthDef(\hit,
		{arg ress = 0, amp =1 ;
			var klank, env;
			klank = Klank.ar(`[{ExpRand(70,10000)}!7, {Rand(0.128,0.700)}!7],BrownNoise.ar(0.7));
			klank = klank;
			env = EnvGen.kr(Env.perc(0.07, ress), doneAction:2);
			klank = CombC.ar(klank, 0.2, Rand(0.01, 0.2));
			Out.ar(0, klank*env.dup*0.0128 * amp);
	}).add;



	SynthDef(\vplaym,
		{arg out = 0, buf = 0, rate = 1, amp = 0.5, pan = 0, pos=0, gate=1;
			var sig, env;
			sig = Pan2.ar(PlayBuf.ar(2,buf,BufRateScale.ir(buf) * rate,1,BufDur.kr(buf)*pos*44100,doneAction:2),pan);
			env = Linen.kr(gate, doneAction: 2);
			sig = sig * env;
			sig = sig * amp;
			Out.ar(out,sig);
	}).add;



	~randomHarm = 1;

	~playSwipe = {
		|msg |

		'swipe'.postln;
		msg.postln;

		if(msg[2] == 'STATE_START', {
			//change harmonization of density in granular synth
			~randomHarm = [rrand(0.5, 2.5), 1].choose;
			~proxy[0] = nil;

			x= rrand(0.3, 0.6);

			y = Synth(\hit, [\ress, x+(7*x), \amp, 0.4]);

			//choose a tuning and scale for the counterclockwise motion


			~scale = Array.new(3);
			~choice = [Scale.chooseFromSelected, Scale.pelog].wchoose([0.7, 0.3]);
			~tuning = Tuning.choose(~choice.pitchesPerOctave);
			~choice.tuning_(~tuning);
	[36,48, 60].do({ |i|
				~scale.add((~choice.degrees +i).midicps)});

	~scale = ~scale.flatten;


		});

	};

	s.sync;


	//////////////////////////////////////////////////////////////////////
	////////////////////////////Functions///////////////////////////////////
	//////////////////////////////////////////////////////////////////////



	~lastDegrees = nil;

	//choose a tuning and pelog scale for counterclockwise motion
	~tuning = Tuning.choose;
	~scale = Array.new(4);
	[36, 48, 60].do({ |i|
		~scale.add((Scale.pelog(~tuning).degrees +i).midicps)});
	~scale = ~scale.flatten;

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

		~proxy.fadeTime = 0.1;


		if(result == 'clockwise',
			{
				//play this sound while
				if(progress == 'STATE_START', {
					~proxy[0] = {
						|amp = 0.3, detune=1, panLFOFreq=0|
						Splay.ar(
							Formlet.ar(
								WhiteNoise.ar*0.05,
								angle.linexp(1, 100, 400, 100) * detune,
								0.1,
								0.5,
								amp
							),
							center: LFTri.kr(panLFOFreq))
					};
				});

				~proxy.set(
					\feeback, radius.linlin(0, 15, 0.5, 1).min(1),
					\size, radius.linlin(0, 15, 0.1, 1).min(1),
					\damp, radius.linlin(0, 15, 1.0, 0),
					\detune, radius.lincurve(0, 20, 1.0, 5, 4),
					\panLFOFreq, radius.linlin(0, 20, 0, 5),
					\amp, radius.linlin(0, 20, 0.3, 0.8)
				);

				if(progress == 'STATE_END',
					{
						~proxy.fadeTime = 0.5;
						~proxy[0] = nil;
				});
			},

			//if its counterclockwise
			{if(progress == 'STATE_START',
				{


					~proxy[0] = nil;
					(angle.linlin(10, 80, ~scale.size, 0)).postln;
					~proxyPl[0] = {
						Splay.ar(
							CombC.ar(
								Pluck.ar(
									WhiteNoise.ar,
									1,
									0.2,
									//pitch determined by angle of circle
									(~scale[(angle.linlin(10, 80, ~scale.size-1, 0))]).reciprocal,
									2,
									rrand(0.2, 0.6),
									rrand(0.5, 1))!2,
								0.2,
								exprand(0.05, 0.2),
								0.5
							)
							,
							center:[0.8, -0.8].choose) * EnvGen.kr(Env.adsr(0.5,0.3, 2, 1), 1, doneAction:2)
					};
		});});



	};


	s.sync;

	//////////////////////////////////////////////////////////////////////
	////////////////////////////Sounds///////////////////////////////////
	//////////////////////////////////////////////////////////////////////

	~dirpath = "11_apr_21".resolveRelative;
	~forest = BuffersFolder.readChannel(~dirpath);
	~forestStereo = BuffersFolder.read(~dirpath);


	//////////////////////////////////////////////////////////////////////
	////////////////////////////Proxys and sounds ///////////////////////////////////
	//////////////////////////////////////////////////////////////////////


	~proxy = NodeProxy.audio(s, 2);
	~proxy.play;
	~proxy.fadeTime = 1;

	~proxyPl = NodeProxy.audio(s, 2);
	~proxyPl.play;



	~proxy[100] = \filter -> { arg in, delay=0.5, damp= 1, size = 1, diff= 0.707, feedback=0.5;
		Greyhole.ar(in, delay, damp, size, diff, feedback)};

	~proxyPl[100] = \filter -> { arg in; FreeVerb.ar(in, 0.8)};




	//////////////////////////////////////////////////////////////////////
	////////////////////////////OSC functions///////////////////////////////////
	//////////////////////////////////////////////////////////////////////


	OSCdef(\gestures, {|msg, time, addr, recvPort|
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

	~start = 'no';

	OSCdef(\wekinator, {
		|msg, time, addr, recvPort|
		msg.postln;


		if(~start == 'no',
			{
				~granular = Synth(\gs, [\buf, ~forest[5], \amp, 1])!5;

				Pdef(\player, Pbind(
					\instrument, \vplaym,
					\buf, ~forestStereo[5],
					\dur, Pwhite(1, 5),
					\amp, 0.15,
					\pos, Pwhite(0, 1),
					\legato, 1.5
				)).play;

				~start = 'yes';


		});

		~granular.do{
			|element, i|
			element.set(
				\sync, msg[1].round,
				\dens, msg[2].linexp(0, 1, 2, 200) * (i+1) * ~randomHarm,
				\grainDur, msg[3].linlin(0, 1, 0.1, 3000.reciprocal),
				\durRand, msg[4]+1,
				\rate, msg[5].lincurve(0, 1, 0.5, 2.5, -3),
				\pos, msg[6],
				\posSpeed, msg[7].linlin(0, 1, 0.5, 1.5),
				\pan, msg[8].linlin(0, 1, -0.8, 0.8),
				\panHz, msg[9].linlin(0, 1, 0, 10),
				\panRand, msg[10].round(0.3),
				\amp, msg[11].lincurve(0.5, 1, 0.5, 3, -2),
				\ampLFO, msg[12],
				\lfoFreq, msg[13].linexp(0, 1, 0.5, 20));
		}
	},
	'/wek/outputs',
	recvPort: 57120
	);



});

s.reboot;

)


