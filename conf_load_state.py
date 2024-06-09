"""
@file    sumoConfigGen.py
@author  Craig Rafter
@date    29/01/2016

Code to generate a config file for a SUMO model.

"""

def sumoConfigGen(modelname='simpleT',
                  configFile='./models/simpleT.sumocfg',
                  exportPath='../results/',
                  CVP=0,
                  stepSize=0.1,
                  run=0,
                  port=8813,
                  seed=23423,
                  route_file='reference.rou.xml',
                  output_path='./test_results/',
                  output_name='reference'):
    routename = modelname
    modelname = modelname.split('_')[0]
    configData = """<configuration>
    <input>
        <net-file value="{model}.net.xml"/>
        <route-files value="{model}.rou.xml"/>
        <!--<gui-settings-file value="gui-settings.cfg"/>-->
        <additional-files value="{model}.det.xml"/>
        <load-state value="states_ubuntu/state_28800.00.sbx"/>
    </input>
    <output>
        <!--<summary-output value="{expPath}summary_R{Nrun:03d}_CVP{cvp:03d}_{model}.xml"/>-->
        <tripinfo-output value="{output_path}trip_info_{output_name}.xml"/>
        <netstate-dump value="{output_path}{output_name}.xml" freq="2"/>
        <netstate-dump.precision value="4"/>
        <!--<vehroute-output value="{expPath}vehroute_R{Nrun:03d}_CVP{cvp:03d}_{model}.xml"/-->
        <!--queue-output value="{expPath}queuedata_R{Nrun:03d}_CVP{cvp:03d}_{model}.xml"/-->
    </output>
    <time>
        <begin value="28800"/>
        <!--stop sim, especially selly oak, after 35 hrs-->
        <step-length value="{stepSz}"/>
    </time>
    <processing>
        <!--TURN OFF TELEPORTING-->
        <time-to-teleport value="-1"/>
        <ignore-junction-blocker value="60"/>
        <!--collision.mingap-factor value="0"/-->
        <!--no-internal-links value="true"/-->
    </processing>
    <random_number>
        <seed value="{seed}"/>
    </random_number>
    <report>
        <no-step-log value="true"/>
        <!--error-log value="logfile.txt"/-->
    </report>
    <traci_server>
        <remote-port value="{SUMOport}"/>
    </traci_server>
""".format(model=modelname,
           route=routename, 
           expPath=exportPath,
           cvp=int(CVP*100),
           stepSz=stepSize,
           Nrun=run,
           SUMOport=port,
           seed=seed,
           route_file=route_file,
           output_path=output_path,
           output_name=output_name)
    
    # write configuration to file
    with open(configFile, 'w') as f:
        f.write(configData)
        f.write("\n</configuration>\n")

def sumoConfigGen_win(modelname='simpleT',
                  configFile='./models/simpleT.sumocfg',
                  exportPath='../results/',
                  CVP=0,
                  stepSize=0.1,
                  run=0,
                  port=8813,
                  seed=23423,
                  route_file='reference.rou.xml',
                  output_path='./test_results/',
                  output_name='reference'):
    routename = modelname
    modelname = modelname.split('_')[0]
    configData = """<configuration>
    <input>
        <net-file value="{model}.net.xml"/>
        <route-files value="{model}.rou.xml"/>
    g   <!--<gui-settings-file value="gui-settings.cfg"/>-->
        <additional-files value="{model}.det.xml"/>
        <load-state value="states_windows/state_28800.00.sbx"/>
    </input>
    <output>
        <!--<summary-output value="{expPath}summary_R{Nrun:03d}_CVP{cvp:03d}_{model}.xml"/>-->
        <!--<tripinfo-output value="{expPath}tripinfo_R{Nrun:03d}_CVP{cvp:03d}_{model}.xml"/>-->
        <netstate-dump value="{output_path}{output_name}.xml" freq="2"/>
        <netstate-dump.precision value="4"/>
        <!--<vehroute-output value="{expPath}vehroute_R{Nrun:03d}_CVP{cvp:03d}_{model}.xml"/-->
        <!--queue-output value="{expPath}queuedata_R{Nrun:03d}_CVP{cvp:03d}_{model}.xml"/-->
    </output>
    <time>
        <begin value="28800"/>
        <!--stop sim, especially selly oak, after 35 hrs-->
        <step-length value="{stepSz}"/>
    </time>
    <processing>
        <!--TURN OFF TELEPORTING-->
        <time-to-teleport value="-1"/>
        <ignore-junction-blocker value="60"/>
        <collision.mingap-factor value="0"/>
        <!--no-internal-links value="true"/-->
    </processing>
    <random_number>
        <seed value="{seed}"/>
    </random_number>
    <report>
        <no-step-log value="true"/>
        <!--error-log value="logfile.txt"/-->
    </report>
    <traci_server>
        <remote-port value="{SUMOport}"/>
    </traci_server>
""".format(model=modelname,
           route=routename, 
           expPath=exportPath,
           cvp=int(CVP*100),
           stepSz=stepSize,
           Nrun=run,
           SUMOport=port,
           seed=seed,
           route_file=route_file,
           output_path=output_path,
           output_name=output_name)
    
    # write configuration to file
    with open(configFile, 'w') as f:
        f.write(configData)
        f.write("\n</configuration>\n")
