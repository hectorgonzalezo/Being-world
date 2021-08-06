
(

//cleaning everything up


~ewiGrp.free;
~ewiFxGrp.free;

Window.closeAll;
Server.killAll;

s.freeAll;

s.reboot;

s.waitForBoot{


	//////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////SynthDefs

	//////////////////////////////////////////////////////////////////////////////////////////////
	SynthDef(\conv, {
		|voc = 1, buf1, buf2, gate=1, atk=0.1, rel=2, amp=1, pan=0, fte1Vol=1, fte2Vol=1, window=0.2, rate=1, pitchDis=0, timeDis=0, rateNoise=0, lowValRate=0.8, highValRate=1.2, out=0, pos=0, mix=(-1),panLFO=0, panNois=0|

		var paneo, sig,fte1, fte2, env, velRate;

		env= Linen.kr(gate,atk,1,rel,2);


		velRate= Select.kr(rateNoise,[
			rate,
			LFNoise1.kr(LFNoise0.kr(0.5).range(0.5,3)).range(rate*lowValRate, rate*highValRate)]);

		fte1 = PlayBuf.ar(2,buf1, startPos:BufFrames.kr(buf1)*pos, loop:1);
		fte2 = PlayBuf.ar(2,buf2,velRate, loop:1);


		fte2 = PitchShift.ar(fte2, window,velRate.reciprocal,pitchDis, timeDis);

		sig = Convolution.ar(fte1*fte1Vol, fte2*fte2Vol ,4096);

		sig = sig*amp*env;

		paneo = Splay.ar(XFade2.ar(sig, fte1,mix),0,center:pan+LFTri.ar(panLFO)*LFNoise1.kr(panNois).unipolar);
		Out.ar(out,paneo);
	}).add;




	SynthDef(\gs, {
		|sync =1, dens=20, grainDur=0.1, durRand=1, buf=0,
		rate=1, pos=0, posSpeed=1, posRand=0,
		grainEnv=(-1), pan=0, panHz=0, panRand=0, gate=1, atk=1, dec=0.3, rel=1, crv=(-1), amp=1, out=0, loopTime=20, ampLFO=0, lfoFreq= 0.5|

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





	SynthDef(\warp1,
		{arg buf=0, channels=2, pos=0, rate=1, size=0.2, env= -1, olap=8, rand=0, interp=4, poslag=0, ratelag=0, out=0, sizelag=0, pan=0, amp=0.5, atk=1,sus=1, rel=2, olapfr=0, olapmul=0.5, gate=1;
			var sig;
			sig = Warp1.ar(2,
				buf,
				Lag.kr(pos,poslag), //pointer
				Lag.kr(rate,ratelag), //freqScale
				Lag.kr(size,sizelag), //windowSize
				env,
				olap+LFNoise1.kr(olapfr,olapmul),                //overlap
				rand,                //windowRandRatio
				interp,              //interpolation
				1);
			env = Linen.kr(gate,atk,1,rel,2);
			sig= LeakDC.ar(sig);
			sig = Limiter.ar(HPF.ar(sig,40),0.9);
			Out.ar(out,Pan2.ar(sig*env,pan,amp));
	}).add;



	SynthDef(\wtabl,
		{arg out = 0, buf = 0, freq = 300, amp = 0.5, pan = 0, atk=0.01, rel=1, pos=0;
			var sig, env;
			sig = Pan2.ar(PlayBuf.ar(1,buf,BufRateScale.ir(buf) *freq,1,BufFrames.kr(buf)*pos,1),pan);
			env = EnvGen.ar(Env.perc(atk,rel),doneAction:2);
			sig = sig * env;
			sig = sig * amp;
			Out.ar(out,sig);
	}).add;




	SynthDef(\stereobuf,
		{
			|out=0, buf = 0, rate=1, trig=1, pos=0, loop=1, amp=1, atk=0.05, rel 0.1, pan=0, gate=1, vol=1|
			var sig, env;

			env = EnvGen.kr(Env.asr(atk,amp,rel),gate, doneAction:2);
			sig= PlayBuf.ar(
				2,
				buf,
				BufRateScale.ir(buf) * rate,
				trig,
				BufFrames.kr(buf)*pos,
				loop
			);
			Out.ar(out,sig * env*vol, pan);
		}
	).add;


	SynthDef(\stereobufShift,
		{
			|out=0, buf = 0, rate=1, trig=1, pos=0, loop=1, amp=1, atk=0.05, rel 0.1, pan=0, gate=1, vol=1, window=0.2, pitchDis=0, timeDis=0, trigTime=0|
			var sig, env;

			env = EnvGen.kr(Env.asr(atk,1,rel),gate, doneAction:2);
			sig= PlayBuf.ar(
				2,
				buf,
				BufRateScale.ir(buf) * rate,
				trig*Pulse.kr(trigTime.reciprocal),
				BufFrames.kr(buf)*pos,
				loop
			);
			sig = PitchShift.ar(sig, window,rate.reciprocal.abs,pitchDis, timeDis);
			Out.ar(out,Splay.ar(sig*amp,1,1,pan));
		}
	).add;



	SynthDef(\monobufShift,
		{
			|out=0, buf = 0, rate=1, trig=1, pos=0, loop=1, amp=1, atk=0.05, rel 0.1, pan=0, gate=1, vol=1, window=0.2, pitchDis=0, timeDis=0, trigTime=0|
			var sig, env;

			env = EnvGen.kr(Env.asr(atk,amp,rel),gate, doneAction:2);
			sig= PlayBuf.ar(
				1,
				buf,
				BufRateScale.ir(buf) * rate,
				trig*Pulse.kr(trigTime.reciprocal),
				BufFrames.kr(buf)*pos,
				loop
			);
			sig = PitchShift.ar(sig, window,rate.reciprocal,pitchDis, timeDis);
			Out.ar(out,Pan2.ar(sig * env*vol, pan));
		}
	).add;






	SynthDef(\bplay,
		{arg out = 0, buf = 0, rate = 1, amp = 0.5, pan = 0, pos = 0, rel=15;
			var sig,env ;
			sig = Mix.ar(PlayBuf.ar(2,buf,BufRateScale.ir(buf) * rate,1,BufDur.kr(buf)*pos*44100,doneAction:2));
			env = EnvGen.ar(Env.linen(0.0,rel,0),doneAction:2);
			sig = sig * env;
			sig = sig * amp;
			Out.ar(out,Pan2.ar(sig,pan));
	}).add;



	SynthDef(\sine,{
		|out=0, freqbeg=1000, freqend=2000, dur=60 vol=0.3, ampbeg=0.1, ampend=0.1, atk=1, gate=1, rel=10, pan=0, noisMul=5 |

		var sig, env;

		sig = SinOsc.ar(
			XLine.kr(freqbeg, freqend, dur),
			0,
			LFNoise1.kr(
				LFNoise0.kr(noisMul).range(noisMul/10,noisMul)
			).range(noisMul/10,1)

		);

		env= EnvGen.kr(Env([0,1,1,0], [atk, dur, rel]),gate, doneAction:2);

		Out.ar(out, Pan2.ar(sig*env*vol, pan));

	}
	).add;



	SynthDef(\recorder, { arg out = 0, buf, run = 1, in, pan=0, vol=1;
		var input;
		input = In.ar(in, 2);
		RecordBuf.ar(input, buf, recLevel: vol, run:run, loop:0, doneAction:2);
		Out.ar(out,input*vol);
	}).add;








	////////////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////FX for general use

	///////////////////////////////////////////////////////////////////////////////////////////////////////



	(
		SynthDef(\delay, {
			arg in=0, out=0, mix=(-0.2), decay=3, delHz=0.2, delMin=0.1, delMax=0.4, gate=1, releaseTime=2, amp=0.8, pos=0, spafr=0, spamul=1, spanois=0, width=2, noismul=1, vol=1, atk=0.5;
			var sig, delay, env;
			sig = In.ar(in);
			delay = CombL.ar(
				sig,
				2,
				delHz.lag2(0.2),//SinOsc.kr([delHz,delHz*0.9]).exprange(delMin, delMax),
				decay

			);
			sig = Mix.ar(XFade2.ar(sig, delay, mix));
			env = EnvGen.kr(Env.asr(atk,amp,releaseTime,1), gate,doneAction:2);

			sig = Pan2.ar(
				sig,
				pos+SinOsc.kr(
					spafr,
					0,
					spamul),
				1
			);
			Out.ar(out,sig*env*vol);
		}).add;
	);




	SynthDef(\shaper,{
		|buf, in, out=0, vol=1, gate=1, pos = 0, atk=0.5, amp=1, releaseTime=1,cutFreq= 10000|
		var source, sig, env;
		source = In.ar(in);
		sig = LPF.ar(Shaper.ar(buf,source),cutFreq);
		env = EnvGen.kr(Env.asr(atk,amp,releaseTime,1), gate,doneAction:2);

		Out.ar(out,sig*env*vol);
	}
	).add;






	SynthDef(\distort, { arg out=0, pregain=40, amp=0.2, gate=1, in, vol =0.8, pos =0, atk=0.5, releaseTime=2, cutFreq=10000;

		var env, source, sig;

		env = EnvGen.kr(Env.asr(atk,1,releaseTime,1), gate,doneAction:2);
		source = LPF.ar((In.ar(in,2) * pregain).distort * amp, cutFreq);

		sig = Splay.ar(source*env*vol);

		Out.ar(out, sig);

	}).add;



	SynthDef(\rev, {
		|in, out, mix=0.33, room =0.55, damp =0.5, atk=0.5, amp=1, releaseTime=1, gate=1, vol=1, pos=0, spamul=1, spanois=0, noismul=1, spafr=0|
		var source, sig, env;
		source = In.ar(in, 2);
		sig = FreeVerb.ar(source,mix,room, damp);
		env = EnvGen.kr(Env.asr(atk,amp,releaseTime,1), gate,doneAction:2);
		Out.ar(out, sig*env*vol);
	}
	).add;





	SynthDef(\grey, {
		arg in, out, vol = 1, time=2, damp=0, size=1, sizech=0, diff = 0.707, fback =0, fbch=0, modDepth=0.1, modFreq=2, atk= 0.02, sus =2, releaseTime = 1, crv = -1, gate=1, pos=0;
		var source, sig, env;
		env = EnvGen.kr(Env.asr(atk,sus,releaseTime,crv),gate, doneAction:2);
		source = In.ar(in);
		sig = Mix.ar(Greyhole.ar(source, time.lag2(1), damp, size.lag2(1) + LFNoise0.kr(sizech),diff.lag2(1), fback.lag2(1), LFNoise1.kr(fbch), modDepth, modFreq));//poner el efecto aqui

		Out.ar(out, sig*env*vol.lag2(1));
	}).add;



	SynthDef(\multiTap, {
		|ich=0, out=0, buf, panlfo = 3, panois = 1, in, gate=1, atk= 0.02, amp =0.8, releaseTime = 2, crv = 1, vol=1, outsPan=2, bpfFreq=440, bpfRq=1|
		var source, sig, env, times, levels;
		times = Array.rand(50, 0.1,1.2).sort;
		levels= Array.rand(50, 0.1,1.0).sort.reverse;
		env = EnvGen.kr(Env.asr(atk,amp,releaseTime,crv),gate, doneAction:2);
		source = In.ar(in,2);

		sig = BPF.ar(MultiTap.ar(times, levels, source.lag(0.02), bufnum:buf),bpfFreq.lag2(0.5), bpfRq);

		sig = Splay.ar(sig,1, center:SinOsc.kr(panlfo*LFNoise1.kr(panois).range(0,1),0).range(-0.8,0.8));
		Out.ar(out, sig * env*vol.lag2(0.5));

	}).add;














	SynthDef(\bitCrush, {
		arg in, out, amp = 1, rate= 44100, bit=8, atk= 0.02, sus =2, releaseTime = 1, crv = -1, gate=1, pos=0, mix=0;
		var source, sig, env;
		env = EnvGen.kr(Env.asr(atk,sus,releaseTime,crv),gate, doneAction:2);
		source = In.ar(in);
		sig = XFade2.ar(source,Decimator.ar(source, rate, bit, Amplitude.kr(source)),mix);
		Out.ar(out, sig * amp * env);
	}).add;








	SynthDef(\limiter, {
		|in, out=0, level=0.9|
		var source, sig;

		source = In.ar(in,2);
		sig = Limiter.ar(source, level, 0.1);

		Out.ar(out, sig);
	}
	).add;







	SynthDef(\lpf, {
		|in, out=0, freq= 1000, lag=0.1|
		var source, sig;

		source = In.ar(in,2);
		sig = LPF.ar(source, freq.lag2(lag));

		Out.ar(out, sig);
	}
	).add;


	SynthDef(\hpf, {
		|in, out=0, freq= 80, lag=0.1|
		var source, sig;

		source = In.ar(in,2);
		sig = HPF.ar(source, freq.lag2(lag));

		Out.ar(out, sig);
	}
	).add;






s.sync;
	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//create GUI
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////

	~x = 600;
	~y = 700;
	~gui = Window.new("Dasein", Rect(
		0,
		(Window.screenBounds.height/2)-(~y/2),//20,
		~x,~y))
	.front
	.alwaysOnTop_(false)
	.background_(Color.grey(0.8));



	~title = StaticText(~gui, Rect(175,15,250,75))
	.string_("Dasein")
	.stringColor_(Color.black)
	.font_(Font("Sakkal Majalla",70, false, false))
	.align_(\center);



	~loopGuis = Array.fill(12,{|i|
		~loop1 = StaticText(~gui, Rect(15,75+(i*50),1000,75))
		.string_(i.midinote[0].asString ++ i.midinote[1].asString ++" loop unactive")
	.stringColor_(Color.blue)
	.font_(Font("Sakkal Majalla",25, false, false))
	.align_(\left);});


	~current = StaticText(~gui, Rect(300,100,1000,75))
	.string_("Current section playing:")
	.stringColor_(Color.black)
	.font_(Font("Sakkal Majalla",25, false, false))
	.align_(\left);

	~secPlay = Button(~gui, Rect(315,175,200,75))
	.states_([
		["None", Color.black, Color.rand],
		["1", Color.black, Color.rand],
		["2", Color.black, Color.rand],
		["3", Color.black, Color.rand],
		["4", Color.black, Color.rand],
		["5", Color.black, Color.rand],
		["End", Color.black, Color.rand],
	])
	.font_(Font("Sakkal Majalla",35, true));



	~meter = ServerMeterView(s,~gui,350@450, 2, 2).start;





	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////start MIDI
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	MIDIClient.init;

	s.sync;
	MIDIIn.connectAll;

	MIDIClient.sources;



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////Groups
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////





	~ewiGrp = Group.new;
	~ewiFxGrp = Group(~ewiGrp, \addAfter);

	~loopGroups = Array.fill(12,{Group.new(~ewiFxGrp, \addBefore)});

	~sentenceGrp = Group(~ewiGrp,\addBefore);

	~sinGrp = Group.new;






	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////Buses
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////


	~ewiFxBusDel = Bus.audio(s,2);
	~ewiFxBusTap = Bus.audio(s,2);
	~ewiFxBusRev = Bus.audio(s,2);



	~recBus = Bus.audio(s,2);


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////PROXYS
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////


	~proxys = Dictionary[
		\drone -> NodeProxy(s).play,
		\chords -> NodeProxy(s).play,
		\stoneSyn -> NodeProxy(s).play,
		\stoneAsyn -> NodeProxy(s).play,
		\rythm -> NodeProxy(s).play,
		\mixedRythm1 -> NodeProxy(s).play,
		\mixedRythm2 -> NodeProxy(s).play,
		\wtablSart -> NodeProxy(s).play,
		\cars -> NodeProxy(s).play,
		\sentencesInf -> NodeProxy(s).play,
		\sentences -> NodeProxy(s).play,
	];





	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////Buffers
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////



	~tapBuff0 = Buffer.alloc(s,s.sampleRate*12,2);



	~tapBuffSt0 = Buffer.alloc(s,s.sampleRate*12);



	~buffers = Array.fill(12, {Buffer.alloc(s, 44100 * 10.0, 2)});





	//booleans and variables
	~sentenceOff=true;
	~vol=0;

	//arrays for looping sections

	~improLoopSynths = Array.fill(12,{nil});
	~playLoopSynths = Array.fill(12,{nil});
	~recordLoopSynths = Array.fill(12,{nil});








	///////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////          FUNCTIONS               ///////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////




	~looperFunc = {

	|note|

		if(~improLoopSynths[note].isPlaying,{
			MIDIdef(\breath,{
				|val|
				if(val==0 && ~improLoopSynths[note].isPlaying,
					{
						{~loopGuis[note].string_(note.midinote[0].asString ++ note.midinote[1].asString ++" loop unactive").stringColor_(Color.blue(0.5));}.defer;
			~improLoopSynths[note].set(\gate, 0)
				})
			})
		},
		{

	//create buffer player with random start position
	~playLoopSynths[note] = Synth(\monobufShift,[
											\loop, 1,
											\out, ~recBus,//output to recorder
											\pan, ~pan,
											\buf, ~improsMono[1],
											\amp, 1,
											\pos, rrand(0,0.8)

										],
	~loopGroups[note]

										);

										NodeWatcher.register(~playLoopSynths[note]);





										//initiate MIDIdefs. The bite controls the synths rate of playback.
										//The thumb controls paning position.
										MIDIdef.cc(\bite,
											{
												|val, nn|
												if(~playLoopSynths[note].isPlaying,
													{~playLoopSynths[note].set(\rate,val.linexp(1,127,1,2));
												});


											},8
										);

										MIDIdef.cc(\upBend,{
											|val, cc|
											~pan = val.linlin(0,127,0,-1);
											if(~playLoopSynths[note].isPlaying,
												{~playLoopSynths[note].set(\pan, val.linlin(0,127,0,-1));
											});
											[val, cc].postln;
										}, 64
										);


										MIDIdef.cc(\downBend,{
											|val, cc|
											~pan = val.linlin(0,127,0,1);
											if(~playLoopSynths[note].isPlaying,
												{~playLoopSynths[note].set(\pan, val.linlin(0,127,0,1));
											});
											[val, cc].postln;
										}, 1
										);



										//start recording
										~recordLoopSynths[note] = Synth(\recorder,[
		\buf, ~buffers[note],
											\in, ~recBus
										],
										~loopGroups[note],
										\addToTail
										);
										~impro1LoopDur=0;

										//this Tdef counts recording time in 0.1 second increments and sets the variable ~impro1LoopDur, that is then used as retrigger for the loop
										Tdef(\counter,{
											inf.do{
												0.1.wait;
												~impro1LoopDur= ~impro1LoopDur+0.1;
											}
										}
										).play;


										//post in GUI
	{~loopGuis[note].string_(note.midinote[0].asString ++ note.midinote[1].asString ++" loop recording").stringColor_(Color.red)}.defer;



										//main Breath controller
										MIDIdef.cc(\breath,
											{
												|val, nn|

												//the stronger the blowing the higher the volume of loop recorded.
												~recordLoopSynths[note].set(\vol,val.linlin(1,127,0,3));
												val.postln;



												if(val==0,//when breath reaches 0 stop recording.
													{
														Tdef(\counter).stop;
														"llega a 0".postln;
														~recordLoopSynths[note].free;
														~playLoopSynths[note].set(\gate, 0, \rel, 1);




														//this is to create a single loop in case 0 is reached multiple times, which happens with the EWIUSB
					if(~improLoopSynths[note].isPlaying,
						{},
															{

																//loop the recording
																~improLoopSynths[note]= Synth(\stereobufShift,[
																	\buf, ~buffers[note],
																	\loop, 1,
																	\amp, 1,
																	\out, ~ewiFxBusRev,//add reverb
																	\trigTime, ~impro1LoopDur;//loop duration
																],
							~loopGroups[note]);
																NodeWatcher.register(~improLoopSynths[note]);
														});


														//free MIDIdefs use for recording
														MIDIdef.cc(\bite).free;



														//this registers the Synth so that ~impro1Loop.isPlaying can be used.



														//post in GUI
					{~loopGuis[note].string_(note.midinote[0].asString ++ note.midinote[1].asString ++" loop active").stringColor_(Color.green(0.5));}.defer;




													}
												);


											},9//listen to CC9
										);
			})
									};




	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Load sounds
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	~impros = Array.new;
	~improsMono = Array.new;

	~carpeta1 = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/impros";

	PathName(~carpeta1).entries.do ({
		arg path;
		~impros = ~impros .add(Buffer.read(s, path.fullPath;));
	});

	~carpeta1b = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/impros";

	PathName(~carpeta1b).entries.do ({
		arg path;
		~improsMono = ~improsMono .add(Buffer.readChannel(s, path.fullPath, channels:[0]));
	});



	~piedras = Array.new;
	~piedrasMono = Array.new;


	~carpeta2 = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/piedras";

	PathName(~carpeta2).entries.do ({
		arg path;
		~piedras = ~piedras .add(Buffer.read(s, path.fullPath;));
	});

	~carpeta2b = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/piedras";

	PathName(~carpeta2b).entries.do ({
		arg path;
		~piedrasMono = ~piedrasMono .add(Buffer.readChannel(s, path.fullPath, channels:[0]));
	});






	~templo = Array.new;
	~temploMono = Array.new;


	~carpeta3 = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/templo";

	PathName(~carpeta3).entries.do ({
		arg path;
		~templo = ~templo .add(Buffer.read(s, path.fullPath;));
	});

	~carpeta3b = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/templo";

	PathName(~carpeta3b).entries.do ({
		arg path;
		~temploMono = ~temploMono .add(Buffer.readChannel(s, path.fullPath, channels:[0]));
	});





	~viaje = Array.new;
	~viajeMono = Array.new;

	~carpeta4 = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/viaje";

	PathName(~carpeta4).entries.do ({
		arg path;
		~viaje = ~viaje .add(Buffer.read(s, path.fullPath;));
	});

	~carpeta4b = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/viaje";

	PathName(~carpeta4b).entries.do ({
		arg path;
		~viajeMono = ~viajeMono .add(Buffer.readChannel(s, path.fullPath, channels:[0]));
	});



	~todos= ~impros++~templo++~piedras++~viaje;
	~todosMono= ~improsMono++~temploMono++~piedrasMono++~viajeMono;




	~sentence = Array.new;

	~sentenceMono = Array.new;

	~carpeta9= PathName(thisProcess.nowExecutingPath).parentPath++"buffers/Sentence";

	PathName(~carpeta9).entries.do ({
		arg path;
		~sentence = ~sentence.add(Buffer.read(s, path.fullPath;));
	});


	~carpeta9b = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/Sentence";
	PathName(~carpeta9b).entries.do ({
		arg path;
		~sentenceMono = ~sentenceMono .add(Buffer.readChannel(s, path.fullPath, channels:[0]));
	});















	/////////////////////////////////////////////////////////////////////////
	///////////////////////////PDEFs
	////////////////////////////////////////////////////////////////////////











	Pdef(\drone,
		Pbind(
			\instrument,\gs,
			\atk, Pdefn(\droneAtk,Pwhite(3,7)),
			\dur, Pdefn(\droneDur,Pwhite(10,20)),
			\legato, Pwhite(1.2,1.4),
			\buf, ~temploMono[1],
			\sync, 1,
			\pos, Pwhite(0.1,0.4),
			\posSpeed, Pwhite(0.01,0.1,inf),
			\dens,([30,Pwhite(30.5,33)] * Prand([1,1.466,1.333],inf)),
			\grainDur, Pwhite(0.1,0.12,inf),
			\amp, Pseg(
				Pseq(Array.rand(6,5,12),inf),
				Pseq(Array.rand(6,5,20).normalizeSum,inf)*(Pkey(\dur)+Pkey(\atk)),
				\lin,
				inf),
			\pan, 0,
			\panHz, Pwhite(0.1,0.5,inf),
			\panRand, Pwhite(0.1,0.5)
		)
	);







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
			\buf, ~temploMono[1],
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




	Pdef(\stoneSyn,
		Pbind(
			\instrument, \gs,
			\dur, Pdefn(\stoneSynDur,Pwhite(15,40)),
			\atk, Pwhite(1,3),
			\rel, Pwhite(5,10),
			\legato, 1.1,
			\buf, ~piedrasMono[0],
			\gate, 1,
			\sus, Pkey(\dur)-Pkey(\atk)-Pkey(\rel),
			\sync,Pwrand([1,0],[0.8,0.2],inf),
			\pos, Pwhite(0.63,0.7),
			\posSpeed, Pseq([0.74,1],inf),
			\posRand, Pwhite(0.1,0.3,inf),
			\grainDur, 0.01,
			\dens, 130,
			\posRand,0.1,
			\rate, Pseq([0.8,1,1.5],inf),
			\amp, Pseg(
				Pseq(Array.rand(3,4.0,10.0),inf),
				Pseq(Array.rand(3,1,6).normalizeSum,inf)*Pkey(\dur),
				\lin,
				inf),
			\pan, -0.65,
			\panHz, 0
		)
	);




	Pdef(\stoneAsyn,
		Pbind(
			\instrument, \gs,
			\dur, Pdefn(\stoneAsynDur, Pwhite(15,40)),
			\atk, 1,
			\rel, 1,
			\legato, 1.1,
			\buf, ~piedrasMono[0],
			\gate, 1,
			\sus, Pkey(\dur)-Pkey(\atk)-Pkey(\rel),
			\sync,0,
			\pos, 0.646,
			\posSpeed, 0.74,
			\posRand, 0.3,
			\grainDur, 0.01,
			\dens, 130,
			\rate, Pwhite(0.05,0.15),
			\pan, 0.65,
			\panHz, 0,
			\amp, Pwhite(10,20)
		)
	);











	Pdef(\wtablSart,
		Pbind(
			\instrument, \wtabl,

			\buf, ~improsMono[3],

			\pos, Pstutter(Pwhite(1,3),Pwhite(0.0001,1.0,inf)),

			\atk, Pwhite(0.1,1.0,inf),

			\rel, Pexprand(3,8,inf),

			\posRand, 0.5,

			\freq, Pwrand([2.5, Pwhite(0.5,2,1)], [0.4,0.6],inf),

			\amp, Pbrown(1.0,1.3,0.05,inf),

			\dur, Pdefn(\wtablSartDur,Pwrand([Pkey(\atk)+ Pkey(\rel), Rest(5)], [0.3, 0.7], inf))

		)
	);






	Pdef(\cars,
		Pbind(
			\instrument, \warp1,
			\buf, ~viaje[0],//Pwrand(Array.fill(3,{|i| ~viaje[i]}),[0.2,0.3,0.5],inf),
			\rate, Pwhite(0.3,1.1).poll,
			\ratelag, 1,
			\olapfr, Prand([0,0.2],inf),
			\olapmul, 0.3,
			\pos, Pexprand(0.0001,1,inf),
			\size, Pwhite(0.05, 0.1).poll*Pdefn(\carsSizeMul,1),
			\olap, Pwhite(2,10).poll,
			\atk, Pwhite(1,3),
			\rel, Pwhite(1,3),
			\dur, Pwrand([Pwhite(5,8
				,1),Rest(1)],[0.7,0.3],10),
			\legato, Pwhite(1.1,1.6),
			\amp, Pwhite(0.1,0.4)
		)
	);












	Pdef(\rythm,
		Pbind(
			\instrument, \wtabl,

			\buf, Pdefn(\bufferRyt,~temploMono[4]),

			\atk, Pwhite(0.001,0.1,inf),

			\rel, Pstutter(Pwhite(1,5),Pwhite(0.5, 2)),

			\dur,Pdefn(\wtablRytDur,
				Pwrand(
					[Pstutter(Pexprand(1,5),
						Pbrown(0.1,0.5,0.1,5)),
					Rest(5)],[0.3,0.7],inf)).poll,

			\freq, Pdefn(\freqWtablRy, Pstutter(
				Pwhite(1,5),
				Pgauss(0.45,0.15,inf))),

			\amp, Pbrown(1.5,4.0,0.5,inf)

		)
	);



	Pdef(\sentenceGran,
		Pbind(
			\instrument, \gs,
			\dur, Pdefn(\sentenceGranDur,Pwhite(5, 10,inf)),
			\legato, 1.8,
			\buf, Pwrand([~sentenceMono[1],~sentenceMono[9],Prand(~sentenceMono,1)],[5,5,15].normalizeSum,inf),
			\sync, 1,
			\pos, Pwhite(0.1,0.3,inf),
			\posSpeed, Pwhite(0.01,0.2,inf)*Pwrand([1,-1],[0.7,0.3],inf).poll,
			\dens, Plprand(5,60),
			\grainDur, Pkey(\dens).reciprocal*Pwhite(0.65,1.35),
			\amp, Pexprand(0.2,0.8),
			\pan, Pwhite(-0.7,0.7)
		)
	);




	Pdef(\mixedRythm,
		Pbind(
			\instrument, \gs,

			\legato, 1,
			\buf, ~temploMono[4],
			\gate, 1,
			\sus, 5,
			\sync,Pwrand([0,1],[0.2,0.8],inf),
			\dur, Pdefn(\mixedRythmDur, Pif(Pkey(\sync).odd, Pwhite(5,8,inf), Pwhite(2,4,inf))).poll,
			\pos, 0.646,
			\posSpeed, Prand([2,3, 4, 5],inf),
			\posRand, Prand([0, Pwhite(0.1,0.2,inf)],inf),
			\grainDur, Pwhite(0.01,0.018,inf),
			\dens, 130,
			\pan, 0,
			\panHz, Pwhite(0.1,1),
			\panRand, Phprand(0.2,1),
			\rate, Pwhite(0.9,1.1),
			\ampLFO, 1,
			\lfoFreq, 0.3,
			\amp, Pwhite(0.6,1.0)
		)
	);



	/////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////TDEFS

	//////////////////////////////////////////////////////////////////////////////////



	//these will be the main 5 sections of the piece, still to work out a scheme.

	Tdef(\section1,{


		{~secPlay.value= 1}.defer;

		{~state = StaticText(~gui, Rect(265,275,300,165))
			.string_("Drone starts playing")
			.stringColor_(Color.black)
			.font_(Font("Sakkal Majalla",50, false, false))
			.align_(\center);}.defer;


		Pdefn(\droneAtk,40);
		Pdefn(\droneDur,60);


		~proxys[\drone].source = Pdef(\drone);
		~proxys[\drone][10] = \filter -> {|in| Greyhole.ar(in,{rrand(0.2,1)},0.2,1,0.1,0.7)};
		~proxys[\drone][20] = \filter -> {|in| Resonz.ar(in, LFNoise1.kr(LFNoise0.kr(0.25).range(0.5,3)).exprange(500,3000))};
		~proxys[\drone].set(\wet20, 0.5);



		rrand(70,100).wait;

		Pdefn(\droneAtk,Pwhite(3,7));
		Pdefn(\droneDur,Pwhite(10,20));


		{~state.string_("Chords start playing")}.defer;


		~proxys[\chords].source = Pdef(\chords);
		~proxys[\chords][10] = \filter -> {|in| FreeVerb.ar(in, 0.4, SinOsc.kr(0.5*LFNoise1.kr(1).range(0.1)).range(0.5,1.5), 0.3)};






	}
	);


	Tdef(\section2,{

		{~secPlay.value= 2}.defer;

		{~state.string_("Drone fade out")}.defer;



		Pdefn(\droneDur, Pwhite(20,40,1));






		20.wait;
		Pdefn(\chordAmp,0.7);


		{~state.string_("Stones start playing")}.defer;



		~proxys[\stoneSyn].source = Pdef(\stoneSyn);
		~proxys[\stoneSyn][10] = \filter -> {|in| Resonz.ar(in*0.1.reciprocal,1000*LFNoise1.kr(0.5).range(0.7,1.3),0.1)};



		~proxys[\stoneAsyn].source = Pdef(\stoneAsyn);
		~proxys[\stoneAsyn][10] = \filter -> {|in| Greyhole.ar(in,{rrand(0.2,1)},0.4, 1, 0.1, 0.8)};
		~proxys[\stoneAsyn].set(\wet10, 0.5);



	}
	);


	Tdef(\section3,{

		{~secPlay.value= 3}.defer;




		{~state.string_("Synchronous stones and chords fade out")}.defer;

		Pdefn(\stoneSynDur, Pwhite(10,15,1));

		Pdefn(\chordsDur, Pwhite(15,20,1));


		7.wait;

		"Timpani and Rythmic chant".postln;
		{~state.string_("Timpani and Rythmic chant start playing")}.defer;



		~proxys[\rythm].source = Pdef(\rythm);
		~proxys[\rythm][10] = \filter -> {|in| LPF.ar((in * 40).distort * 0.2,5000)};
		~proxys[\rythm].set(\wet10, 0.5);


		~proxys[\rythm][20] = \filter -> {|in| FreeVerb.ar(in, 0.5,1)};




		~proxys[\mixedRythm1].source = Pdef(\mixedRythm);
		~proxys[\mixedRythm1][10] = \filter -> {|in| HPF.ar(in,200)};

		~proxys[\mixedRythm2].source = Pdef(\mixedRythm);
		~proxys[\mixedRythm2][10] = \filter -> {|in| DynKlank.ar(`[
			Array.exprand(6,3000,10000),
			Array.rand(6,0.005,0.03),
			Array.fill(6,{|i| i.linexp(0,6,1,5)*0.1})],
		in,
		{LFNoise0.kr(9.reciprocal).range(0.8,1.2).lag2(9)}!6
		)};



		30.wait;

		"StoneAsyn fades out".postln;
		{~state.string_("Asynchronous stones fade out")}.defer;

		Pdefn(\stoneAsynDur, Pwhite(10,15,1));

		rrand(40,70).wait;


		"Sartales".postln;
		{~state.string_("String of seeds starts playing")}.defer;


		~proxys[\wtablSart].source = Pdef(\wtablSart);
		~proxys[\wtablSart][10] = \filter -> {|in| CombC.ar(in, 0.2, Amplitude.ar(in).range(0.2,0.01), 1)};
		~proxys[\wtablSart].set(\wet10, 0.15);
	}
	);


	Tdef(\section4,{

		{~secPlay.value= 4}.defer;


		"Timpani and Rythmic chant fade out".postln;

		{~state.string_("Timpani and Rythmic chant fade out")}.defer;

		Pdefn(\wtablRytDur, Pwhite(10,20,1));
		Pdefn(\mixedRythmDur, Pwhite(10,20,1));

		rrand(10,15).wait;
		"Sartales fade out".postln;
		{~state.string_("Improvise!")}.defer;

		Pdefn(\wtablSartDur, Pwhite(20,50,1));



		inf.do({
			{~state.stringColor_(Color.rand)}.defer;
			0.5.wait;
		})




	}
	);



	Tdef(\section5,{
		{~secPlay.value= 5;}.defer;

		Ppar(Pdef(\sentenceGran)!2,inf).play;



		{~state.string_("Sentences start playing")}.defer;


	}
	);


	Tdef(\section6,{
		{~secPlay.value= 6;}.defer;

		Pdefn(\sentenceGranDur, Pwhite(3,7,1));

		{~state.string_("The end")}.defer;


	}
	);








	///////////////////////////////////////////////////////////////////////////////////////
	/////////////////////MIDI Wind Controller main patch
	//////////////////////////////////////////////////////////////////////////////////////

	/*
	The controller is able to output 5 octaves worth of note numbers. These octaves are divided into different roles.

	*/
	~gate=0;

	MIDIdef.noteOn(\EWI,
		{
			|val, nn|

			case(
				////////--------------------------------------------------------------------------------------------------------------------
				//first octave
				//triggers the main sections of the piece
				////////--------------------------------------------------------------------------------------------------------------------

				{nn<48},

				{

					switch(nn,
						34, {if(Tdef(\section1).isPlaying,{},{Tdef(\section1).play})}, //Bb
						35, {if(Tdef(\section1).isPlaying,{Tdef(\section1).stop;Tdef(\section2).play})}, //B
						36, {if(Tdef(\section2).isPlaying,{Tdef(\section2).stop;Tdef(\section3).play})}, //C
						37, {if(Tdef(\section3).isPlaying,{Tdef(\section3).stop;Tdef(\section4).play})}, //C#
						38, {if(Tdef(\section4).isPlaying,{Tdef(\section4).stop;Tdef(\section5).play})}, //D
						39, {if(Tdef(\section5).isPlaying,{Tdef(\section5).stop;Tdef(\section6).play})},  //Eb

					)



				},


				////////--------------------------------------------------------------------------------------------------------------------
				//second octave
				/*
				various events:

				C- Individual stone pulses with effects

				C#- Cars pattern
				D- 5 pairs of high sinusoids

				Eb- Start playing last sentence from "Being and Time" by Martin Heidegger
				E- Stop playing sentence

				F- Play sentence once

				F#- Start playing sentence convolved with percussive improvisaion
				G- Stop playing convolved sentence

				G#- All words in sentence simultaneously

				*/
				////////--------------------------------------------------------------------------------------------------------------------
				{nn>=48 && nn<60},

				{


					MIDIdef.noteOff(\EWIoff,
						{

							"EWIOFF".postln;
							if(a.isPlaying,{a.set(\gate,0)});
							~gate=0;
						}
					);

					~delayEwi.set(\mix, -1);


					MIDIdef.cc(\downBend).free;
					MIDIdef.cc(\upBend).free;
					MIDIdef.cc(\bite).free;
					MIDIdef.cc(\breath).free;

					switch(nn,
						//C
						48, {

							~ewiGrp.deepFree;

							if(~gate==0,{
								a = Synth(\stereobuf,[
									\buf, ~piedras[5],
									\sus, 0.5,
									\loop, 1,
									\amp, 2,
									\vol, ~vol,
									\out, ~ewiFxBusTap
								],
								~ewiGrp

								);






								NodeWatcher.register(a);

								~gate=1;
							},
							{nil}
							);




							MIDIdef.cc(\breath,
								{
									|val, nn|

									if(a.isPlaying,{

										~vol =  val.linexp(0,127,0.3,10);
										a.set(\vol, val.linexp(0,127,0.3,10),\rate,val.linexp(0,127,0.5,1.5));
										val.postln;

										if(val ==0, {a.set(\gate, 0);
											/*
											~multiTapEwi.free;
											~distortEwi.free;
											*/
											MIDIdef.cc(\downBend).free;
											MIDIdef.cc(\upBend).free;
											MIDIdef.cc(\bite).free;
											MIDIdef.cc(\breath).free;
										});

									}
									)
								},9
							);

							MIDIdef.cc(\downBend, {
								|val|
								~multiTapEwi.set(\panlfo, val.linlin(0,127,1,20));

								val.postln;},1);


							MIDIdef.cc(\upBend,{
								|val, cc|
								~multiTapEwi.set(\bpfFreq, val.linexp(0,127,400,1000)) ;
								val.postln;
							}, 64
							);

							MIDIdef.cc(\bite, {
								|val|


								~multiTapEwi.set(\vol, val.linexp(0,127,1,100),\bpfRq,val.linexp(0,127,1,0.01))
							},8);
						},

						//C#,
						49,{


							~proxys[\cars].source = Pdef(\cars);
		~proxys[\wtablSart][10] = \filter -> {|in| CombC.ar(in, 0.2, Amplitude.ar(in).range(0.2,0.01), 1)};




						},

						//D

						50, {
							10.do({
								~fund=exprand(rrand(500,3000), 10000);
								2.do(
									{
										|i|
										Synth(\sine, [
											\freqbeg, ~fund,
											\freqend, rrand(~fund,~fund* [1.003,1.003.reciprocal].choose),
											\dur, 20,
											\atk, rrand(0.5,10),
											\rel, rrand(5,15),
											\vol, 0.01,
											\out, i,
											\group, ~sinGrp,
											\pan, rrand(-1,1)

										]
										)
									}
								)
							}
							);



							MIDIdef.cc(\bite, {
								|val|


								~sinGrp.set(\noisMul, val.linlin(0,127,5,100));
								val.postln;
							},8);
						},




						//D#
						51, {

							MIDIdef.cc(\upBend,{
								|val|
								Pdefn(\sentenceRate, val.linlin(0,127,1.0,2.0));
								val.postln;
							},64
							);

							MIDIdef.cc(\downBend,{
								|val|
								Pdefn(\stretchMul, val.linlin(0,127,1.0,2.0));
								val.postln;
							},1
							);

							MIDIdef.cc(\bite,{
								|val|
								if(val==127,{

									~senScrambleInf = ~sentence.scramble;
									~senDurInf= Array.new(~senScrambleInf.size);
									~senScrambleInf.size.do({
										|i|
										if(i==9,{
											~senDurInf.add((~senScrambleInf[i].numFrames/44100*2))
										},
										{
											~senDurInf.add(~senScrambleInf[i].numFrames/44100)}
										)
									}
									);

									Pdefn(\sentencInfBuf,Pseq(~senScrambleInf,inf));
									Pdefn(\sentenceInfDur, Pdefn(\sentenceGate,Pseq(~senDurInf,inf)*Pkey(\rate).abs.reciprocal));

									"sentence Scrambled".postln;

								});
							},8
							);

							if(~sentenceOff,{
								~senScrambleInf = ~sentence.scramble;
								~senDurInf= Array.new(~senScrambleInf.size);
								~senScrambleInf.size.do({
									|i|
									if(i==9,{
										~senDurInf.add((~senScrambleInf[i].numFrames/(44100*2)))
									},
									{
										~senDurInf.add(~senScrambleInf[i].numFrames/44100)}
									)
								}
								);

								Pdef(\sentencesInf,
									Pbind(\instrument, \stereobufShift,
										\loop, 0,
										\buf, Pdefn(\sentencInfBuf,Pseq(~senScrambleInf,inf)),
										\rate, Pdefn(\sentenceRate,Pwrand([-1,1],[0.2,0.8],inf).poll)*Pdefn(\stretchMul,1).reciprocal,
										\pos, Pif(Pkey(\rate)<0,0.9999,0).poll,
										\dur,Pdefn(\sentenceInfDur, Pdefn(\sentenceGate,Pseq(~senDurInf,inf)*Pkey(\rate).abs.reciprocal)),
										\stretch, Pwhite(0.8,1.2)*Pdefn(\stretchMul,1),
										\pan, Pwhite(-1.0,1.0,inf),
										\amp, Pwhite(0.3,0.85)
									);
								);



								~proxys[\sentencesInf].source = Pdef(\sentencesInf);
								~proxys[\sentencesInf][10] = \filter -> {|in| FreeVerb.ar(in,0.5,0.6)};



									~sentenceOff=false;
								}
							)

							},

							//E

							52, {

								Pdefn(\sentenceGate, Pseq(~senDurInf,1)*Pkey(\rate).abs.reciprocal); //para apagar
								~sentenceOff = true;
							},


							//F

							53, {



								~senScramble = ~sentence.scramble;
								~senDur= Array.new(~senScramble.size);
								~senScramble.size.do({
									|i|
									if(i==9,{
										~senDur.add((~senScramble[i].numFrames/44100*60))
									},
									{
										~senDur.add(~senScramble[i].numFrames/44100)}
									)
								}
								);

								Pdef(\sentences,
									Pbind(\instrument, \stereobufShift,
										\loop, 0,
										\buf, Pseq(~senScramble,1),
										\rate, Pdefn(\sentenceRate,1.5*Pwrand([-1,1],[0.1,0.9],~senScramble.size).poll),
										\pos, Pif(Pkey(\rate)<0,0.9999,0).poll,
										\dur, Pdefn(\sentenceGate,Pseq(~senDur,inf)*Pkey(\rate).abs.reciprocal),
										\pan, Pwhite(-1,1,inf),
										\amp, Pwhite(0.23,0.7),
									)
								);


							~proxys[\sentences].source = Pdef(\sentencesInf);
							~proxys[\sentences][10] = \filter -> {|in| Greyhole.ar(in,1, 0.5,1,2, 0.8)};

							},





							//F#
							54, {



								MIDIdef.cc(\downBend, {
									|val|
									~sentenceConv.set(\panLFO, val.linlin(0,127,0.0,0.3),\panNois, val.linlin(0,127,0.0,0.6));

									val.postln;},1);


								MIDIdef.cc(\upBend,{
									|val, cc|
									~revEwi.set(\mix, val.linexp(0,127,0.3,1)) ;
									val.postln;
								}, 64
								);

								MIDIdef.cc(\bite,{
									|val|
									if(val==127,{



										~senScramble2 = ~sentence.scramble;
										~senDur2= Array.new(~senScramble2.size);
										~senScramble2.size.do({
											|i|
											if(i==9,{
												~senDur2.add((~senScramble2[i].numFrames/44100*rrand(1.0,2.0)))
											},
											{
												~senDur2.add(~senScramble2[i].numFrames/44100)}
											)
										}
										);

										b = Pseq(~senScramble2, inf).asStream;

										"Sentence Scrambled".postln;
									}
									);

								},8
								);

								if(~sentenceConv.isPlaying,{nil},
									{

										~senScramble2 = ~sentence.scramble;
										~senDur2= Array.new(~senScramble2.size);
										~senScramble2.size.do({
											|i|
											if(i==9,{
												~senDur2.add((~senScramble2[i].numFrames/44100*rrand(1.0,2.0)))
											},
											{
												~senDur2.add(~senScramble2[i].numFrames/44100*rrand(1,3))}
											)
										}
										);




										Tdef(\oracion,{
											~sentenceConv=Synth(
												\conv, [
													\buf1, ~impros[1],
													\buf2, ~senScramble2[0],
													\amp, 0.55,
													\fte1Vol, 1,
													\rateNoise,1,
													\out, ~ewiFxBusRev,
													\pos, rrand(0,0.8),
													\mix, -1],~sentenceGrp);

											b = Pseq(~senScramble2, inf).asStream;

											NodeWatcher.register(~sentenceConv);


											inf.do({
												~senDur2.do({
													|i|

													i.wait;

													~sentenceConv.set(\buf2, b.next);
													i.postln;
												})
											});


										}).play;
								});


							},



							//G
							55,
							{

								Tdef(\oracion).stop;
								~sentenceConv.set(\gate,0);


							},




							//G#
							56,
							{
								Pdef(\sentenceRr,
									Pbind(
										\instrument, \gs,
										\atk, 0.01,
										\dur, 5,
										\rel, 1,
										\legato, 1,
										\buf, ~sentenceMono[2],
										\pos, 0.2,
										\posSpeed, Pdefn(\RrPosSpeed,0.0605),
										\dens, 31,
										\grainDur, 0.938,
										\amp, Pwhite(0.1,0.35, 1),
										\pan, 0,
										\out, 0
									).play;

									MIDIdef.cc(\downBend,{
										|val|
										Pdefn(\RrPosSpeed, val.linlin(0,127,0.0605,0.01));
										val.postln;
									},1
									);

								)
							},
						)


					},






					////////--------------------------------------------------------------------------------------------------------------------
					//third octave


					//delayed rock pulses. Delay time changes with high note number.
					////////--------------------------------------------------------------------------------------------------------------------
					{nn>=60 && nn<72},

					{

						~delayEwi.set(\mix, -0.2, \out, ~ewiFxBusRev);


						if(~gate==0,
							{
								if(a.isPlaying,
									{
										//Delaytime is lowered with each higher note in the octave.
										~delayEwi.set(\delHz, nn.linexp(60,71, 0.15,0.015))},
									{

										~ewiGrp.deepFree;
										"sale".postln;
										a = Synth(\stereobuf,[
											\buf, ~piedras[5],
											\sus, 0.5,
											\rate, 0.5,
											\loop, 1,
											\amp, 1,
											\vol, ~vol,
											\out, ~ewiFxBusDel
										],
										~ewiGrp

										);

										NodeWatcher.register(a);

										~delayEwi.set(\delHz, nn.linexp(60,71, 0.15,0.015));

									};
								);

								~gate=1;

							}
						);


						MIDIdef.cc(\downBend).free;
						MIDIdef.cc(\upBend).free;
						MIDIdef.cc(\bite).free;
						MIDIdef.cc(\breath).free;
						MIDIdef.cc(\EWIoff).free;

						//rate of playback is modulated via the breath.
						MIDIdef.cc(\breath,
							{
								|val|

								~vol = val.linlin(0,127,0,3);
								if(a.isPlaying,{
									a.set(\vol, val.linlin(0,127,0,3));
									val.postln;
								})

							},9
						);

						MIDIdef.cc(\bite,
							{
								|val|

								if(a.isPlaying,{
									a.set(\rate, val.expexp(1,127,0.5,1.5));
									val.postln;
								})

							},8
						);



						//controllers Upbend controls the decay value of the delay, functioning as a note sustainer up to a point.
						MIDIdef.cc(\upBend,{
							|val, cc|
							~delayEwi.set(\decay, val.linlin(0,127,1,10));
							val.postln;



						}, 64
						);


						//Downbend controls an LFO paning the sound in the stereo field
						MIDIdef.cc(\downBend, {
							|val|
							~delayEwi.set(\spafr, val.linlin(0,127,0,20)) ;
						},1);





					},


					////////--------------------------------------------------------------------------------------------------------------------
					//fourth octave

					//Loop playback and recording of some recordings of my trip.
					////////--------------------------------------------------------------------------------------------------------------------

					{nn>=72 && nn<84},

					{

					~looperFunc.value(nn%12);


					},






					////////--------------------------------------------------------------------------------------------------------------------
					//fifth octave

					//granulated recording, almost accordion sound. Pitches mapped to note numbers.
					////////--------------------------------------------------------------------------------------------------------------------
					{nn>=84},

					{

						~delayEwi.set(\out, 0);

						MIDIdef.cc(\breath).free;
						MIDIdef.cc(\bite).free;
						MIDIdef.cc(\upBend).free;
						MIDIdef.cc(\downBend).free;
						MIDIdef.cc(\EWIoff).free;





						if(~gate==0,
							{
								"sale".postln;


								if(a.isPlaying,{a.set(\dens,(nn-48).midicps)},
									{
										~ewiGrp.deepFree;

										a= Synth( \gs,[
											\atk, 0.1,
											\rel, 0.001,
											\buf, ~temploMono[1],
											\pos, 0.3,
											\posSpeed, 0.0605,
											\dens, (nn-48).midicps,
											\grainDur, 0.2,
											\rate, 0.5,
											\amp, ~amp,
											\pan, 0,
											\out, ~ewiFxBusDel
										], ~ewiGrp);
										NodeWatcher.register(a);

										~delayEwi.set(\mix, -1, \delHz, rrand(0.2,0.5));


								});

								~gate=1;
							};

						);


						MIDIdef.cc(\breath,
							{
								|val|

								~amp = val.linlin(0,127,0.05,15);
								if(a.isPlaying,{
									a.set(\amp, val.linlin(0,127,0.05,15));

									val.postln;
								})

							},9
						);

						MIDIdef.cc(\bite,
							{
								|val|



								if(a.isPlaying,{
									a.set(\posSpeed, val.linexp(0,127,0.0605,0.1));
									val.postln;
								},
								)

							},8
						);



						//controllers Upbend controls the decay value of the delay, functioning as a note sustainer up to a point.
						MIDIdef.cc(\upBend,{
							|val, cc|

							~delayEwi.set(\decay, val.linexp(0,127,0.01,inf), \mix, val.linlin(0,127,-1,0.5));
							val.postln;



						}, 64
						);


						//Downbend controls an LFO paning the sound in the stereo field
						MIDIdef.cc(\downBend, {
							|val|
							a.set(\grainDur, val.linexp(0,127,0.3,0.05)) ;
						},1);

					}







				);

			};
		);



		//these two MIDIdefs are to clear any remaining synth playing either when breath is stopped or on NoteOff

		MIDIdef.cc(\breathGate,
			{
				|val, nn|

				if(a.isPlaying,{

					if(val ==0, {
						"BREATHGATE".postln;
						a.set(\gate, 0)});

				});


				~gate=0;


			},11
		);

		MIDIdef.noteOff(\EWIoff,
			{

				"EWIOFF".postln;
				if(a.isPlaying,{a.set(\gate,0)});
				~gate=0;
			}
		);

















		//General purpose effects, always running.

		~upBend=0;





		~delayEwi= Synth(\delay, [
			\in, ~ewiFxBusDel,
			\out, 0
		],
		~ewiFxGrp,
		\addToTail);


		~multiTapEwi= Synth(\multiTap, [
			\in, ~ewiFxBusTap,
			\vol, 3,
			\out, 0,
			\bpfRq,2,
			\buf, ~tapBuff0
		],
		~ewiFxGrp,
		\addToTail);


		~revEwi= Synth(\rev,[
			\in, ~ewiFxBusRev,
			\mix, 0.3,
			\out, 0
		],
		~ewiFxGrp,
		\addToTail
		);



		1.wait;

		s.plotTree(0.1);
		s.scope(2);
		s.freqscope;


	};



)
