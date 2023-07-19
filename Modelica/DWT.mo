package DWT
  extends Modelica.Icons.Package;

  model SystemData
    import Modelica.Units.SI;
    import Modelica.Constants.pi;
    parameter DWT.Units.ApparentPower Sbase = 100 annotation(
      Dialog(group = "Base Quantities"));
    parameter SI.Frequency fb = 60 annotation(
      Dialog(group = "Base Quantities"));
    parameter SI.AngularVelocity wb = 2*pi*fb;
    annotation(
      defaultComponentName = "data",
      defaultComponentPrefixes = "inner",
      missingInnerMessage = "The System object is missing, please drag it on the top layer of your model",
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, extent = {{-80, 70}, {80, -70}}), Text(extent = {{-60, 40}, {60, -40}}, textString = "System")}),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end SystemData;

  package Units
    extends Modelica.Icons.Package;
    type PerUnit = Real(unit = "pu");
    operator record CPerUnit = Complex(redeclare PerUnit re, redeclare PerUnit im);
    type ActivePower = Real(final quantity = "Power", final unit = "MW");
    type ApparentPower = Real(final quantity = "Power", final unit = "MVA");
    type ReactivePower = Real(final quantity = "Power", final unit = "Mvar");
    type Voltage = Real(final quantity = "Voltage", final unit = "kV");
    type Current = Real(final quantity = "Current", final unit = "kA");
  end Units;

  package Math
    extends Modelica.Icons.FunctionsPackage;

    function polar2cart
      extends Modelica.Icons.Function;
      import Modelica.ComplexMath.exp;
      import Modelica.ComplexMath.j;
      import Modelica.Constants.pi;
      input Real mag "Absolute value of the complex";
      input Modelica.Units.SI.Angle phase "Phase angle of the complex";
      output Complex z "Resultant complex number";
    algorithm
      z := mag*exp(j*phase*pi/180);
    end polar2cart;
  end Math;

  package Circuit
    extends Modelica.Icons.Package;

    package Interfaces
      extends Modelica.Icons.InterfacesPackage;

      connector PositivePin
        import DWT.Units;
        Units.CPerUnit v "Positive node voltage";
        flow Units.CPerUnit i "Sum of currents flowing into node";
        annotation(
          defaultComponentName = "pin_p",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-160, 110}, {40, 50}}, lineColor = {0, 0, 255}, textString = "%name")}));
      end PositivePin;

      connector NegativePin
        import DWT.Units;
        Units.CPerUnit v "Negative node voltage";
        flow Units.CPerUnit i "Sum of currents flowing into node";
        annotation(
          defaultComponentName = "pin_n",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-40, 110}, {160, 50}}, textString = "%name", lineColor = {0, 0, 255})}));
      end NegativePin;

      model Bus
        import DWT.Units;
        import Modelica.Units.SI;
        DWT.Circuit.Interfaces.PositivePin p(v.re(start = 1.0)) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-100, -100}, {100, 100}}, rotation = 0), iconTransformation(origin = {-2, -20}, extent = {{-15, -150}, {15, 150}}, rotation = 0)));
        Units.PerUnit V(start = 1.0);
        SI.Angle angle(start = 0);
      equation
        V^2 = p.v.re^2 + p.v.im^2;
        p.v.im = p.v.re*tan(angle);
        p.i = Complex(0);
        annotation(
          Icon(graphics = {Rectangle(origin = {-7, 3}, extent = {{1, 97}, {13, -105}}), Text(origin = {-5, 163}, lineColor = {0, 0, 255}, extent = {{-83, 41}, {83, -41}}, textString = "%name")}));
      end Bus;

      partial model SeriesComponent
        import DWT.Units;
        Units.CPerUnit v "Voltage drop accros this circuit element.";
        Units.CPerUnit i "Current flowing from pin 'p' to pin 'n'";
        PositivePin p annotation(
          Placement(visible = true, transformation(origin = {-46, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-96, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        NegativePin n annotation(
          Placement(visible = true, transformation(origin = {46, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        v = p.v - n.v;
        i = p.i;
        p.i + n.i = Complex(0);
      end SeriesComponent;

      partial model ShuntComponent
        import DWT.Units;
        DWT.Circuit.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Units.CPerUnit v "Node voltage across this shunt element.";
        Units.CPerUnit i "Current flowing towards the reference node.";
      equation
        v = p.v;
        i = p.i;
      end ShuntComponent;
    end Interfaces;

    package Sources
      extends Modelica.Icons.SourcesPackage;

      model VoltageSource
        extends DWT.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        import DWT.Math.polar2cart;
        //
        parameter DWT.Units.PerUnit magnitude = 1.0;
        parameter Modelica.Units.NonSI.Angle_deg angle = 0.0;
        DWT.Units.CPerUnit S;
      equation
        v = polar2cart(magnitude, angle);
        S = -v*conj(i);
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}), Line(origin = {-73, 0}, points = {{-13, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}})}, coordinateSystem(initialScale = 0.1, extent = {{-100, -100}, {100, 100}})));
      end VoltageSource;

      model ControlledVoltageSource
        extends DWT.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        import DWT.Math.polar2cart;
        //
        DWT.Units.CPerUnit S;
        Modelica.ComplexBlocks.Interfaces.ComplexInput u annotation(
          Placement(visible = true, transformation(origin = {-62, -48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {6, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        v = u;
        S = -v*conj(i);
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}, endAngle = 360), Line(origin = {-73, 0}, points = {{-25, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}})}, coordinateSystem(initialScale = 0.1)));
      end ControlledVoltageSource;
    end Sources;

    package Basic
      extends Modelica.Icons.Package;

      model SeriesImpedance
        extends DWT.Circuit.Interfaces.SeriesComponent;
        parameter DWT.Units.PerUnit r = 0.0;
        parameter DWT.Units.PerUnit x = 0.0;
      equation
        v = Complex(r, x)*i;
        annotation(
          Icon(graphics = {Rectangle(origin = {1, -1}, extent = {{-61, 35}, {61, -35}}), Line(origin = {-73, 0}, points = {{13, 0}, {-13, 0}}), Line(origin = {76, 0}, points = {{-14, 0}, {14, 0}})}, coordinateSystem(initialScale = 0.1)));
      end SeriesImpedance;

      model Ground
        DWT.Circuit.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        p.v.re = 0;
        p.v.im = 0;
        annotation(
          Diagram,
          Icon(graphics = {Line(origin = {0, -40}, points = {{-60, 0}, {60, 0}, {60, 0}}), Line(origin = {0, -60}, points = {{-40, 0}, {40, 0}}), Line(origin = {0, -80}, points = {{-20, 0}, {20, 0}}), Line(origin = {0, -15}, points = {{0, 5}, {0, -25}})}, coordinateSystem(initialScale = 0.1)));
      end Ground;
    end Basic;
  end Circuit;

  package WindTurbine
    extends Modelica.Icons.Package;

    model DFIG
      parameter DWT.WindTurbine.Interfaces.DWTData smData;
      DWT.WindTurbine.MachineModel.MIT mit(Hm = smData.convData.Hm, Llr = smData.convData.Llr, Lls = smData.convData.Lls, Lm = smData.convData.Lm, Rr = smData.convData.Rr, Rs = smData.convData.Rs) annotation(
        Placement(visible = true, transformation(origin = {-4, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.WindTurbine.TurbineModel.TURBINA TURBINA(Dtm = smData.convData.Dtm, Ht = smData.convData.Ht, Ktm = smData.convData.Ktm, N = smData.N, Pb = 1e6*smData.MVAb, R = smData.R, Wrmb = smData.Wrmb, kb = smData.kb, par = smData.par, tb = smData.tb) annotation(
        Placement(visible = true, transformation(origin = {-36, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.WindTurbine.ConversorModel.CONVERSOR conversor(Ceq = smData.convData.Ceq, Kc = smData.Kc) annotation(
        Placement(visible = true, transformation(origin = {30, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.WindTurbine.Sensors.CurrentSensor currentSensor annotation(
        Placement(visible = true, transformation(origin = {0, -6}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      DWT.WindTurbine.Sensors.CurrentSensor currentSensor1 annotation(
        Placement(visible = true, transformation(origin = {56, -6}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      DWT.WindTurbine.Sensors.PowerSensor powerSensor annotation(
        Placement(visible = true, transformation(origin = {64, 14}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      DWT.WindTurbine.Sensors.VoltageSensor voltageSensor annotation(
        Placement(visible = true, transformation(origin = {72, -36}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor tac annotation(
        Placement(visible = true, transformation(origin = {-20, -6}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-54, 14}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.Circuit.Interfaces.PositivePin pin_DFIG annotation(
        Placement(visible = true, transformation(origin = {76, 14}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.WindTurbine.ControlModel.CONTROL control(Ceq = smData.convData.Ceq, Idgref = smData.Idgref, Kc = smData.Kc, Lc = smData.convData.Lc, Lm = smData.convData.Lm, Lr = smData.convData.Llr + smData.convData.Lm, Ls = smData.convData.Lls + smData.convData.Lm, Qgref = smData.Qgref, Vccref = smData.Vccref, Vw_max = smData.Vw_max, Vw_min = smData.Vw_min, Vw_nom = smData.Vw_nom, Vw_wmax = smData.Vw_wmax, Vw_wmin = smData.Vw_wmin, Wrm_min = smData.Wrm_min, Wrm_nom = smData.Wrm_nom, fileNameR2 = smData.fileNameR2, fileNameR4 = smData.fileNameR4, kiIdg = smData.kiIdg, kiIdr = smData.kiIdr, kiIqg = smData.kiIqg, kiIqr = smData.kiIqr, kiPLL = smData.kiPLL, kiQs = smData.kiQs, kiVcc = smData.kiVcc, kiWrm = smData.kiWrm, kpIdg = smData.kpIdg, kpIdr = smData.kpIdr, kpIqg = smData.kpIqg, kpIqr = smData.kpIqr, kpPLL = smData.kpPLL, kpVcc = smData.kpVcc, kpWrm = smData.kpWrm, tableNameR2 = smData.tableNameR2, tableNameR4 = smData.tableNameR4) annotation(
        Placement(visible = true, transformation(origin = {16, -32.5}, extent = {{-26, -6.5}, {26, 6.5}}, rotation = 0)));
    equation
      connect(currentSensor.pin_p, conversor.outR) annotation(
        Line(points = {{4, -6}, {18.6, -6}}));
      connect(conversor.outG, currentSensor1.pin_p) annotation(
        Line(points = {{41.4, -6}, {52, -6}}));
      connect(Vw, TURBINA.Vw) annotation(
        Line(points = {{-54, 14}, {-40, 14}}, color = {0, 0, 127}));
      connect(TURBINA.flange_Eixo, mit.eixo) annotation(
        Line(points = {{-30, 14}, {-4, 14}}));
      connect(tac.flange, mit.eixo) annotation(
        Line(points = {{-20, -2}, {-20, 14}, {-4, 14}}));
      connect(currentSensor.pin_n, mit.pin_rotor) annotation(
        Line(points = {{-4, -6}, {-4, 8}}, color = {0, 0, 255}));
      connect(conversor.outVcc, control.Vccmed) annotation(
        Line(points = {{30, -14}, {30, -24}}, color = {0, 0, 127}));
      connect(pin_DFIG, voltageSensor.pin_p) annotation(
        Line(points = {{76, 14}, {72, 14}, {72, -32}}, color = {0, 0, 255}));
      connect(conversor.Mr, control.Mr) annotation(
        Line(points = {{24, -16}, {24, -24}}, color = {85, 170, 255}));
      connect(conversor.Mg, control.Mg) annotation(
        Line(points = {{36, -16}, {36, -24}}, color = {85, 170, 255}));
      connect(pin_DFIG, powerSensor.pin_n) annotation(
        Line(points = {{76, 14}, {68, 14}}, color = {0, 0, 255}));
      connect(currentSensor1.pin_n, powerSensor.pin_p) annotation(
        Line(points = {{60, -6}, {60, 14}}, color = {0, 0, 255}));
      connect(mit.pin_estator, powerSensor.pin_p) annotation(
        Line(points = {{7, 14}, {60, 14}}));
  connect(currentSensor.outI, control.Irmed) annotation(
        Line(points = {{0, -10}, {0, -24}}, color = {0, 0, 127}, thickness = 0.5));
  connect(TURBINA.Beta, control.Beta) annotation(
        Line(points = {{-34, 4}, {-32, 4}, {-32, -33}, {-11, -33}}, color = {0, 0, 127}));
  connect(control.Wmed, tac.w) annotation(
        Line(points = {{-11, -29}, {-20, -29}, {-20, -10}}, color = {0, 0, 127}));
  connect(control.Vw, Vw) annotation(
        Line(points = {{-11, -36}, {-54, -36}, {-54, 14}}, color = {0, 0, 127}));
  connect(control.Igmed, currentSensor1.outI) annotation(
        Line(points = {{43, -29}, {56, -29}, {56, -10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control.Vt, voltageSensor.outV) annotation(
        Line(points = {{43, -36}, {68, -36}}, color = {0, 0, 127}, thickness = 0.5));
  connect(control.Qmed, powerSensor.outS[2]) annotation(
        Line(points = {{44, -32}, {64, -32}, {64, 10}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.001),
        Diagram(graphics = {Rectangle(origin = {-35, 14}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-9, 22}, {9, -22}}), Rectangle(origin = {-3, 13}, fillColor = {255, 255, 255}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-13, 13}, {13, -13}}), Rectangle(origin = {31, -7}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-17, 11}, {17, -11}}), Text(origin = {-4, 32}, extent = {{-20, 4}, {20, -4}}, textString = "MODELO
MIT"), Text(origin = {-36, 42}, extent = {{-22, 4}, {22, -4}}, textString = "MODELO
TURBINA"), Text(origin = {30, 10}, extent = {{-24, 4}, {24, -4}}, textString = "MODELO
CONVERSOR"), Rectangle(origin = {16, -32}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-32, 10}, {32, -10}}), Text(origin = {17, -48}, extent = {{-23, 4}, {23, -4}}, textString = "MODELO 
CONTROLE")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-3, -3}, textColor = {0, 0, 255}, extent = {{-67, 35}, {67, -35}}, textString = "DFIG")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end DFIG;

    package Interfaces
      extends Modelica.Icons.InterfacesPackage;

      record DWTData
        extends Modelica.Icons.Record;
        import DWT.Units;
        import SI = Modelica.Units.SI;
        parameter Units.ApparentPower MVAs = 2 "System base power" annotation(
          Dialog(group = "Base Quatities"));
        parameter Units.ApparentPower MVAb = 2 "Machine base power" annotation(
          Dialog(group = "Base Quatities"));
        parameter Integer Nmaq = 1 "Number of parallel machines" annotation(
          Dialog(group = "Base Quatities"));
        parameter Real Wb = 1 "Frequency base" annotation(
          Dialog(group = "Base Quatities"));
        parameter Real Wrmb = 2*Wb/Polos "Frequency mech base" annotation(
          Dialog(group = "Base Calculed"));
        // Elec data:
        parameter Units.PerUnit Rs = 0.01 "Stator resistance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Rr = 0.01 "Rotor resistance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lls = 0.1 "Stator leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Llr = 0.08 "Rotor leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lm = 3.0 "Stator magnetizing inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lc = 0 "Filter inductance" annotation(
          Dialog(group = "Electrical Data"));
        // Mech data:
        parameter Integer Polos = 4 "Number of polos by machine" annotation(
          Dialog(group = "Mechanical Data"));
        parameter SI.Time Hm = 0.524 "Machine inertia constant" annotation(
          Dialog(group = "Mechanical Data"));
        parameter Units.PerUnit Dm = 0.0 "Viscous friction of the machine" annotation(
          Dialog(group = "Mechanical Data"));
        parameter SI.Time Ht = 4.2 "Turbine inertia constant" annotation(
          Dialog(group = "Mechanical Data"));
        parameter Units.PerUnit Dt = 0.0 "Viscous friction of the turbine" annotation(
          Dialog(group = "Mechanical Data"));
        parameter Units.PerUnit Ktm = 0.3 "Shaft elasticity constant" annotation(
          Dialog(group = "Mechanical Data"));
        parameter Units.PerUnit Dtm = 1.5 "Viscous friction of the shaft" annotation(
          Dialog(group = "Mechanical Data"));
        // Aero data:
        parameter Units.PerUnit N = 111.5 "Gear ratio" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit R = 37.5 "Helix radius" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit par = 1.225 "Air density" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit Vw_min = 4 "Min wind speed" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit Vw_nom = 12 "Nom wind speed" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit Vw_max = 25 "Max wind speed" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit Vw_wmin = 8.0713 "Min wind for rotor speed" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit Vw_wmax = 11.1757 "Max wind for rotor speed" annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Wrm_min = 0.8052 "Min speed reference" annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Wrm_nom = 1.115 "Nom speed reference" annotation(
          Dialog(group = "Turbine Data"));
        // Converter data:
        parameter Units.PerUnit Kc = 0.5*(sqrt(3)/sqrt(2))*(1400/690) "Modulation base" annotation(
          Dialog(group = "Conversor Data"));
        parameter Units.PerUnit Ceq = 35.897 "Capacitor of converter" annotation(
          Dialog(group = "Conversor Data"));
        // Control pitch data:
        parameter Units.PerUnit tb = 0.7143 "Controler pitch angle" annotation(
          Dialog(group = "Pitch Control Data"));
        parameter Units.PerUnit kb = 1.1905 "Controler by pitch angle" annotation(
          Dialog(group = "Pitch Control Data"));
        // Control setings:
        parameter Units.PerUnit kiWrm = 6.04672 "ki by Wrm" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpWrm = 15.1168 "kp by Wrm" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiQs = 1 "ki by Qs" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiVcc = 574.352 "ki by Vcc" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpVcc = 287.176 "kp by Vcc" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIqr = 1131.354839 "ki by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIqr = 28.273871 "kp by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIdr = 1131.354839 "ki by Idr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIdr = 28.273871 "kp by Idr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIqg = 256.0 "ki by Iqg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIqg = 6.4 "kp by Iqg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIdg = 256.0 "ki by Idg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIdg = 6.4 "kp by Idg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiPLL = 200 "Integral constant by PLL" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpPLL = 100 "Proporcional constant by PLL" annotation(
          Dialog(group = "Control setings"));
        // Reference data:
        parameter Units.PerUnit Vccref = 1 "Reference voltage by Vcc" annotation(
          Dialog(group = "Reference data"));
        parameter Units.PerUnit Idgref = 0 "Reference current by Idg" annotation(
          Dialog(group = "Reference data"));
        parameter Units.PerUnit Qgref = 0 "Reference of reactive power" annotation(
          Dialog(group = "Reference data"));
        // Lookup table data:
        parameter String tableNameR2 = "omega" "Table name on file or in function usertab" annotation(
          Dialog(group = "Lookup table definition"));
        parameter String fileNameR2 = "NoName" "File where matrix is stored" annotation(
          Dialog(group = "Lookup table definition", loadSelector(filter = "MATLAB MAT-files (*.mat)", caption = "Open file in which table is present")));
        parameter String tableNameR4 = "beta" "Table name on file or in function usertab" annotation(
          Dialog(group = "Lookup table definition"));
        parameter String fileNameR4 = "NoName" "File where matrix is stored" annotation(
          Dialog(group = "Lookup table definition", loadSelector(filter = "MATLAB MAT-files (*.mat)", caption = "Open file in which table is present")));

        record ConvertedData
          parameter Units.PerUnit Rs = 0.01 annotation(
            Dialog(group = "Electrical Data"));
          parameter Units.PerUnit Rr = 0.01 annotation(
            Dialog(group = "Electrical Data"));
          parameter Units.PerUnit Lls = 0.1 annotation(
            Dialog(group = "Electrical Data"));
          parameter Units.PerUnit Llr = 0.08 annotation(
            Dialog(group = "Electrical Data"));
          parameter Units.PerUnit Lm = 3.0 annotation(
            Dialog(group = "Electrical Data"));
          parameter Units.PerUnit Lc = 0 annotation(
            Dialog(group = "Electrical Data"));
          // Mechanical:
          parameter Units.PerUnit Hm = 0.524 annotation(
            Dialog(group = "Mechanical Data"));
          parameter Units.PerUnit Ht = 4.2 annotation(
            Dialog(group = "Mechanical Data"));
          parameter Units.PerUnit Ktm = 0.3 annotation(
            Dialog(group = "Mechanical Data"));
          parameter Units.PerUnit Dtm = 1.5 annotation(
            Dialog(group = "Mechanical Data"));
          // Conversor:
          parameter Units.PerUnit Ceq = 35.897 annotation(
            Dialog(group = "Conversor Data"));
        end ConvertedData;

        ConvertedData convData(Rs = MVAs/MVAb/Nmaq*Rs, Rr = MVAs/MVAb/Nmaq*Rr, Lls = MVAs/MVAb/Nmaq*Lls, Llr = MVAs/MVAb/Nmaq*Llr, Lm = MVAs/MVAb/Nmaq*Lm, Lc = MVAs/MVAb/Nmaq*Lc, Hm = Nmaq*MVAb/MVAs*Hm, Ht = Nmaq*MVAb/MVAs*Ht, Ktm = Nmaq*MVAb/MVAs*Ktm, Dtm = Nmaq*MVAb/MVAs*Dtm, Ceq = MVAs/MVAb/Nmaq*Ceq);
        annotation(
          defaultComponentName = "smData",
          defaultVariability = "Parameter");
      end DWTData;
    end Interfaces;

    package TurbineModel
      extends Modelica.Icons.Package;

      function CP
        extends Modelica.Icons.FunctionsPackage;
        input Real lambda "Velocidade específica da turbina";
        input Real Beta "Ângulo de pitch";
        output Real Cp "Coeficiente de potência";
      protected
        Real alpha;
        constant Real R = 37.5;
        constant Real C[9] = {0.22, 116, 0.4, 0, 0, 5, 12.5, 0.08, 0.035};
      algorithm
        alpha := 1/(1/(lambda + Beta*C[8]) - C[9]/(Beta^3 + 1));
        Cp := C[1]*(C[2]/alpha - C[3]*Beta - C[4]*Beta^C[5] - C[6])*exp(-C[7]/alpha);
      end CP;

      model TURBINA
        import SI = Modelica.Units.SI;
        import pi = Modelica.Constants.pi;
        // Valores base para conversão pu:
        parameter SI.Power Pb = 2e6 annotation(
          Dialog(group = "Base Data"));
        parameter SI.AngularVelocity Wrmb = 60*pi annotation(
          Dialog(group = "Base Data"));
        // Parâmetros da turbina:
        parameter SI.Time Ht = 4.2 annotation(
          Dialog(group = "Turbine Data"));
        parameter SI.Density par = 1.225 annotation(
          Dialog(group = "Turbine Data"));
        parameter SI.Length R = 37.5 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real N = 111.5 annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit Ktm = 0.3 annotation(
          Dialog(group = "Mechanical Data"));
        parameter Units.PerUnit Dtm = 1.5 annotation(
          Dialog(group = "Mechanical Data"));
        // Parâmetros associados a dinâmica de Beta:
        parameter Real kb = 0.7143 annotation(
          Dialog(group = "Control Calculed"));
        parameter Real tb = 1.1905 annotation(
          Dialog(group = "Control Calculed"));
        // Declaração de variáveis:
        DWT.Units.PerUnit Ttur, Ptur, Wtur;
        Real cp "%";
        // Conexões de interface:
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_Eixo annotation(
          Placement(visible = true, transformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vw annotation(
          Placement(visible = true, transformation(origin = {-118, 48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Beta annotation(
          Placement(visible = true, transformation(origin = {-118, -58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {28, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Mechanics.Rotational.Components.Inertia inertia_tur(J = 2*Ht) annotation(
          Placement(visible = true, transformation(origin = {32, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
          Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Components.SpringDamper eixo(a_rel(fixed = false), c = Ktm, d = Dtm, phi_rel(fixed = false), phi_rel0(displayUnit = "rad"), w_rel(fixed = false)) annotation(
          Placement(visible = true, transformation(origin = {68, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator tfBeta(initType = Modelica.Blocks.Types.Init.SteadyState, k = kb) annotation(
          Placement(visible = true, transformation(origin = {52, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction tfdBeta(a = {tb, 1}, b = {1}, initType = Modelica.Blocks.Types.Init.SteadyState) annotation(
          Placement(visible = true, transformation(origin = {-4, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add(k2 = -1) annotation(
          Placement(visible = true, transformation(origin = {-32, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 10) annotation(
          Placement(visible = true, transformation(origin = {24, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      initial equation
        der(Wtur) = 0;
        der(eixo.phi_rel) = 0;
      equation
// Conexões com flange da turbina:
        Ttur = torque.tau;
        Wtur = inertia_tur.w;
// Expressão da aerodinâmica:
        cp = CP(R*(Wtur*Wrmb/N)/Vw, tfBeta.y);
        Ptur = 0.5*(par*pi*R^2*cp*Vw^3)/Pb;
// Obtendo o conjugado da turbina:
        Ttur = Ptur/Wtur;
        connect(torque.flange, inertia_tur.flange_b) annotation(
          Line(points = {{8, 0}, {22, 0}}));
        connect(inertia_tur.flange_a, eixo.flange_a) annotation(
          Line(points = {{42, 0}, {58, 0}}));
        connect(eixo.flange_b, flange_Eixo) annotation(
          Line(points = {{78, 0}, {104, 0}}));
        connect(add.u2, tfBeta.y) annotation(
          Line(points = {{-44, -68}, {-46, -68}, {-46, -82}, {64, -82}, {64, -62}}, color = {0, 0, 127}));
        connect(tfdBeta.y, limiter.u) annotation(
          Line(points = {{7, -62}, {11, -62}}, color = {0, 0, 127}));
        connect(limiter.y, tfBeta.u) annotation(
          Line(points = {{35, -62}, {39, -62}}, color = {0, 0, 127}));
        connect(add.y, tfdBeta.u) annotation(
          Line(points = {{-21, -62}, {-17, -62}}, color = {0, 0, 127}));
  connect(Beta, add.u1) annotation(
          Line(points = {{-118, -58}, {-44, -58}, {-44, -56}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(origin = {35, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-15, 6}, {15, -6}}), Rectangle(origin = {3, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-17, 20}, {17, -20}}), Ellipse(origin = {-10, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-20, 20}, {20, -20}}), Polygon(origin = {0, 80}, lineColor = {0, 0, 255}, points = {{6, -60}, {0, -60}, {0, 120}, {20, 0}, {20, 0}, {6, -60}}), Polygon(origin = {0, -110}, lineColor = {0, 0, 255}, points = {{0, 90}, {0, -90}, {20, 28}, {6, 90}, {0, 90}, {0, 90}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end TURBINA;
    end TurbineModel;

    package MachineModel
      extends Modelica.Icons.Package;

      model MIT
        import SI = Modelica.Units.SI;
        //  Declarando variáveis do problema:
        Units.PerUnit Wrm "Velocidade do rotor [pu]";
        Units.PerUnit Te "Conjugado elétrico [pu]";
        Units.PerUnit Vqs, Vds "Componentes de tensão no estator [pu]";
        Units.PerUnit Vqr, Vdr "Componentes de tensão no rotor [pu]";
        Units.PerUnit Iqs, Ids "Componentes de corrente no estator [pu]";
        Units.PerUnit Iqr, Idr "Componentes de corrente no rotor [pu]";
        Units.PerUnit fqs, fds "Componentes de fluxo no estator [pu]";
        Units.PerUnit fqr, fdr "Componentes de fluxo no rotor [pu]";
        //  Parâmetros MIT (2 MW, 690V, 60Hz):
        parameter DWT.Units.PerUnit Rs = 0.01 "Resistência do estator [pu]" annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Rr = 0.01 "Resistência do rotor [pu]" annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Lls = 0.1 "Dispersão do estator [pu]" annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Llr = 0.08 "Dispersão do estator [pu]" annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Lm = 3.0 "Magnetização [pu]" annotation(
          Dialog(group = "Eletrical Data"));
        parameter SI.Time Hm = 0.52 "Constante de Inércia [s]" annotation(
          Dialog(group = "Eletrical Data"));
        // Interfaces:
        Modelica.Mechanics.Rotational.Interfaces.Flange_a eixo annotation(
          Placement(visible = true, transformation(origin = {-20, 16}, extent = {{-2, -2}, {2, 2}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Interfaces.PositivePin pin_estator annotation(
          Placement(visible = true, transformation(origin = {-21, -1}, extent = {{-3, -3}, {3, 3}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Interfaces.NegativePin pin_rotor annotation(
          Placement(visible = true, transformation(origin = {21, -1}, extent = {{-3, -3}, {3, 3}}, rotation = 0), iconTransformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Components.Inertia inertia_maq(J = 2*Hm) annotation(
          Placement(visible = true, transformation(origin = {-2.22045e-16, 16}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
          Placement(visible = true, transformation(origin = {12, 16}, extent = {{2, -2}, {-2, 2}}, rotation = 0)));
      initial equation
        der(Wrm) = 0;
        inertia_maq.phi = 0;
        der(fqr) = 0;
        der(fdr) = 0;
      equation
//  Entradas externas:
        Vqs = pin_estator.v.re;
        Vds = pin_estator.v.im;
        Iqs = pin_estator.i.re;
        Ids = pin_estator.i.im;
        Vqr = pin_rotor.v.re;
        Vdr = pin_rotor.v.im;
        Iqr = pin_rotor.i.re;
        Idr = pin_rotor.i.im;
//  Conexões de interface:
        Te = torque.tau;
        Wrm = inertia_maq.w;
//  Equações de fluxo e tensão do estator:
        fqs = Lls*Iqs + Lm*(Iqs + Iqr);
        fds = Lls*Ids + Lm*(Ids + Idr);
        Vqs = Rs*Iqs + fds;
// + der(fqs);// negligenciada
        Vds = Rs*Ids - fqs;
// + der(fds);// negligenciada
//  Equações de fluxo e tensão do rotor:
        fqr = Llr*Iqr + Lm*(Iqs + Iqr);
        fdr = Llr*Idr + Lm*(Ids + Idr);
        Vqr = Rr*Iqr + (1 - Wrm)*fdr + der(fqr);
        Vdr = Rr*Idr - (1 - Wrm)*fqr + der(fdr);
//  Representação do conjugado elétrico:
        Te = fds*Iqs - fqs*Ids;
        connect(eixo, inertia_maq.flange_a) annotation(
          Line(points = {{-20, 16}, {-4, 16}}));
        connect(inertia_maq.flange_b, torque.flange) annotation(
          Line(points = {{4, 16}, {10, 16}}));
        annotation(
          experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.0001),
          Diagram(coordinateSystem(extent = {{-20, 20}, {20, -20}})),
          Icon(graphics = {Bitmap(extent = {{20, 0}, {20, 0}}), Ellipse(lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-100, 100}, {100, -100}}), Ellipse(lineColor = {0, 0, 255}, extent = {{-50, 50}, {50, -50}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end MIT;
    end MachineModel;

    package ConversorModel
      extends Modelica.Icons.Package;

      model CONVERSOR
        // Parâmetros do conversor:
        parameter DWT.Units.PerUnit Kc = 0.5*(sqrt(3)/sqrt(2))*(1400/690) "Relação de transformação (PWM)" annotation(
          Dialog(group = "Eletrical Data"));
        parameter Units.PerUnit Ceq = 35.897 "Capacitância [pu]" annotation(
          Dialog(group = "Conversor Data"));
        // Interfaces do conversor:
        DWT.Circuit.Interfaces.PositivePin outR annotation(
          Placement(visible = true, transformation(origin = {-84, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-114, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Sources.ControlledVoltageSource VSR annotation(
          Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.ComplexBlocks.Interfaces.ComplexInput Mr annotation(
          Placement(visible = true, transformation(origin = {-30, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-60, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.ComplexBlocks.Interfaces.ComplexInput Mg annotation(
          Placement(visible = true, transformation(origin = {30, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {60, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealOutput outVcc annotation(
          Placement(visible = true, transformation(origin = {-22, -18}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -74}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        DWT.Circuit.Interfaces.PositivePin outG annotation(
          Placement(visible = true, transformation(origin = {108, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {114, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Sources.ControlledVoltageSource VSG annotation(
          Placement(visible = true, transformation(origin = {70, 10}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(visible = true, transformation(origin = {20, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C = Ceq) annotation(
          Placement(visible = true, transformation(origin = {20, 14}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Electrical.Analog.Sources.SignalCurrent IccRSC annotation(
          Placement(visible = true, transformation(origin = {-40, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(
          Placement(visible = true, transformation(origin = {-8, 14}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Electrical.Analog.Sources.SignalCurrent IccGSC annotation(
          Placement(visible = true, transformation(origin = {40, 14}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
      initial equation
        der(capacitor.v) = 0;
      equation
// Sinais de corrente para garantir o balanço de potência:
        VSR.S.re = IccRSC.i*capacitor.v;
        VSG.S.re = IccGSC.i*capacitor.v;
// Sinais de referencia para as fontes CA:
        VSR.v = Kc*capacitor.v*Mr;
        VSG.v = Kc*capacitor.v*Mg;
// Conexões do conversor:
        connect(ground.p, capacitor.n) annotation(
          Line(points = {{20, -4}, {20, 4}}, color = {0, 0, 255}));
        connect(voltageSensor.p, capacitor.p) annotation(
          Line(points = {{-8, 24}, {20, 24}}, color = {0, 0, 255}));
        connect(ground.p, voltageSensor.n) annotation(
          Line(points = {{20, -4}, {-8, -4}, {-8, 4}}, color = {0, 0, 255}));
        connect(voltageSensor.v, outVcc) annotation(
          Line(points = {{-19, 14}, {-22, 14}, {-22, -18}}, color = {0, 0, 127}));
        connect(VSR.p, outR) annotation(
          Line(points = {{-70, 20.2}, {-81, 20.2}, {-81, 20}, {-84, 20}}, color = {0, 0, 255}));
        connect(IccGSC.n, capacitor.p) annotation(
          Line(points = {{40, 24}, {20, 24}}, color = {0, 0, 255}));
        connect(ground.p, IccGSC.p) annotation(
          Line(points = {{20, -4}, {40, -4}, {40, 4}}, color = {0, 0, 255}));
        connect(ground.p, IccRSC.p) annotation(
          Line(points = {{20, -4}, {-40, -4}, {-40, 4}}, color = {0, 0, 255}));
        connect(voltageSensor.p, IccRSC.n) annotation(
          Line(points = {{-8, 24}, {-40, 24}}, color = {0, 0, 255}));
        connect(VSG.p, outG) annotation(
          Line(points = {{70, 20}, {108, 20}, {108, 22}}, color = {0, 0, 255}));
        annotation(
          Icon(graphics = {Rectangle(origin = {-60, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-40, 80}, {40, -80}}), Rectangle(origin = {60, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-40, 80}, {40, -80}}), Line(origin = {-0.5, 35}, points = {{-19.5, 25}, {20.5, 25}, {20.5, 25}, {0.5, 25}, {0.5, -25}, {-9.5, -25}, {10.5, -25}, {10.5, -25}}, color = {0, 0, 255}), Line(origin = {-0.5, -35}, points = {{-19.5, -25}, {20.5, -25}, {0.5, -25}, {0.5, 35}, {-9.5, 35}, {10.5, 35}, {10.5, 35}}, color = {0, 0, 255}), Line(origin = {-85.43, -0.5}, points = {{-7, 0.5}, {7, 0.5}, {7, 20.5}, {7, -19.5}, {7, -19.5}}, color = {0, 0, 255}), Line(origin = {-61.5647, 0.185321}, points = {{-8.66454, 20.0191}, {-8.66454, -19.9809}, {-8.66454, 0.019071}, {-8.66454, 10.0191}, {11.3355, 30.0191}, {-8.66454, 10.0191}, {-8.66454, -9.9809}, {11.3355, -29.9809}, {-8.66454, -9.9809}, {-8.66454, -9.9809}}, color = {0, 0, 255}), Line(origin = {-31.5099, 0.62111}, rotation = 180, points = {{18.0358, 30}, {8.0358, 30}, {8.03576, 6}, {4.03576, 6}, {8.03576, -4}, {12.0358, 6}, {8.03576, 6}, {12.0358, 6}, {8.03576, -4}, {4.03576, -4}, {12.0358, -4}, {8.03576, -4}, {8.03576, -28}, {8.0358, -30}, {18.0358, -30}}, color = {0, 0, 255}), Line(origin = {88.5539, 1.90625}, rotation = 180, points = {{18.0358, 32}, {8.03576, 32}, {8.03576, 6}, {4.03576, 6}, {8.03576, -4}, {12.0358, 6}, {8.03576, 6}, {12.0358, 6}, {8.03576, -4}, {4.03576, -4}, {12.0358, -4}, {8.03576, -4}, {8.03576, -28}, {8.0358, -28}, {18.0358, -28}}, color = {0, 0, 255}), Line(origin = {55.83, -0.02}, points = {{-8.66454, 20.0191}, {-8.66454, -19.9809}, {-8.66454, 0.019071}, {-8.66454, 10.0191}, {15.3355, 30.0191}, {-8.66454, 10.0191}, {-8.66454, -9.9809}, {15.3355, -29.9809}, {-8.66454, -9.9809}, {-8.66454, -9.9809}}, color = {0, 0, 255}), Line(origin = {32.17, -0.5}, points = {{-7, 0.5}, {7, 0.5}, {7, 20.5}, {7, -19.5}, {7, -19.5}}, color = {0, 0, 255})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
      end CONVERSOR;
    end ConversorModel;

    package ControlModel
      extends Modelica.Icons.Package;

      package Funcoes
        extends Modelica.Icons.FunctionsPackage;

        model T
          Modelica.Blocks.Interfaces.RealInput u[2] annotation(
            Placement(visible = true, transformation(origin = {32, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {-112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
            Placement(visible = true, transformation(origin = {-98, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput theta annotation(
            Placement(visible = true, transformation(origin = {-22, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -112}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        equation
          y[1] = cos(theta)*u[1] + sin(theta)*u[2];
          y[2] = (-sin(theta)*u[1]) + cos(theta)*u[2];
          annotation(
            Diagram,
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}, color = {0, 0, 255}), Text(origin = {19, -81}, textColor = {0, 0, 255}, extent = {{-81, 19}, {81, -19}}, textString = "qd"), Text(origin = {-20, 81}, textColor = {0, 0, 255}, extent = {{-80, 19}, {80, -19}}, textString = "Sys")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
        end T;

        model iT
          Modelica.Blocks.Interfaces.RealInput u[2] annotation(
            Placement(visible = true, transformation(origin = {32, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
            Placement(visible = true, transformation(origin = {-98, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Blocks.Interfaces.RealInput theta annotation(
            Placement(visible = true, transformation(origin = {-8, -94}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        equation
          y[1] = cos(theta)*u[1] - sin(theta)*u[2];
          y[2] = sin(theta)*u[1] + cos(theta)*u[2];
          annotation(
            Diagram,
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}, color = {0, 0, 255}), Text(origin = {40, -43}, lineColor = {0, 0, 255}, extent = {{-82, 35}, {82, -35}}, textString = "qd"), Text(origin = {-30, 52}, lineColor = {0, 0, 255}, extent = {{60, -28}, {-60, 28}}, textString = "Sys")}));
        end iT;

        model PI_ASTROM
          parameter Real kp = 1, ki = 1;
          Modelica.Blocks.Math.Add comparador(k2 = -1) annotation(
            Placement(visible = true, transformation(origin = {-28, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Math.Add saida(k2 = -1) annotation(
            Placement(visible = true, transformation(origin = {34, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Continuous.Integrator integrador(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki) annotation(
            Placement(visible = true, transformation(origin = {2, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Math.Gain proporcional(k = kp) annotation(
            Placement(visible = true, transformation(origin = {-28, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput r annotation(
            Placement(visible = true, transformation(origin = {-82, 56}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput m annotation(
            Placement(visible = true, transformation(origin = {-82, 22}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -112}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Blocks.Interfaces.RealOutput y annotation(
            Placement(visible = true, transformation(origin = {68, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
          connect(integrador.y, saida.u1) annotation(
            Line(points = {{13, 50}, {21, 50}, {21, 34}}, color = {0, 0, 127}));
          connect(comparador.y, integrador.u) annotation(
            Line(points = {{-17, 50}, {-10, 50}}, color = {0, 0, 127}));
          connect(saida.y, y) annotation(
            Line(points = {{46, 28}, {68, 28}}, color = {0, 0, 127}));
          connect(r, comparador.u1) annotation(
            Line(points = {{-82, 56}, {-40, 56}}, color = {0, 0, 127}));
          connect(m, comparador.u2) annotation(
            Line(points = {{-82, 22}, {-54, 22}, {-54, 44}, {-40, 44}}, color = {0, 0, 127}));
          connect(m, proporcional.u) annotation(
            Line(points = {{-82, 22}, {-40, 22}}, color = {0, 0, 127}));
          connect(proporcional.y, saida.u2) annotation(
            Line(points = {{-16, 22}, {22, 22}}, color = {0, 0, 127}));
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "PI")}));
        end PI_ASTROM;

        model Limiter
          import arg = Modelica.ComplexMath.arg;
          import absC = Modelica.ComplexMath.abs;
          parameter Real maxMod = 1;
          //Real absU, faseU, absY, faseY;
          Modelica.Blocks.Interfaces.RealInput u[2] annotation(
            Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
            Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Nonlinear.Limiter Limiter1(uMax = maxMod) annotation(
            Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Nonlinear.Limiter Limiter(uMax = maxMod) annotation(
            Placement(visible = true, transformation(origin = {0, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
/*absU = absC(u1.y);
        faseU = arg(u1.y);
        absY = if absU < maxMod then absU else maxMod;
        faseY = faseU;
        absY = absC(y1.u);
        faseY = arg(y1.u);*/
          connect(Limiter1.y, y[2]) annotation(
            Line(points = {{12, -30}, {70, -30}, {70, 0}, {106, 0}}, color = {0, 0, 127}));
          connect(u[2], Limiter1.u) annotation(
            Line(points = {{-106, 0}, {-60, 0}, {-60, -30}, {-12, -30}}, color = {0, 0, 127}));
          connect(Limiter.y, y[1]) annotation(
            Line(points = {{12, 22}, {70, 22}, {70, 0}, {106, 0}}, color = {0, 0, 127}));
          connect(u[1], Limiter.u) annotation(
            Line(points = {{-106, 0}, {-60, 0}, {-60, 22}, {-12, 22}}, color = {0, 0, 127}));
          annotation(
            Icon(graphics = {Line(points = {{-80, -70}, {-50, -70}, {50, 70}, {80, 70}}, color = {0, 0, 255}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{0, 90}, {-8, 68}, {8, 68}, {0, 90}}), Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}}), Line(points = {{-90, 0}, {68, 0}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{90, 0}, {68, -8}, {68, 8}, {90, 0}}), Line(visible = false, points = {{50, 70}, {80, 70}}, color = {255, 0, 0}), Line(points = {{0, -90}, {0, 68}}, color = {192, 192, 192}), Line(visible = false, points = {{-80, -70}, {-50, -70}}, color = {255, 0, 0})}),
            experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
        end Limiter;

        model Modulation
          parameter Units.PerUnit Kc = 0.5*(sqrt(3)/sqrt(2))*(1400/690) "Relação de transformação (PWM)";
          Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
            Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput u[2] annotation(
            Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
            Placement(visible = true, transformation(origin = {14, -114}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        equation
          y[1] = u[1]*Kc*Vccmed;
          y[2] = u[2]*Kc*Vccmed;
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "PWM")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
        end Modulation;

        model PLL
          parameter Real kp = 10, ki = 10;
          DWT.WindTurbine.ControlModel.Funcoes.T park annotation(
            Placement(visible = true, transformation(origin = {-42, 0}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
          Modelica.Blocks.Continuous.PI pi(T = kp/ki, initType = Modelica.Blocks.Types.Init.SteadyState, k = kp) annotation(
            Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Continuous.Integrator integrator(y(fixed = false)) annotation(
            Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput Delta annotation(
            Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput Vqds[2] annotation(
            Placement(visible = true, transformation(origin = {70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
          Modelica.Blocks.Interfaces.RealInput Vt[2] annotation(
            Placement(visible = true, transformation(origin = {-88, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        equation
          connect(Vt, park.u) annotation(
            Line(points = {{-88, 0}, {-58, 0}}, color = {0, 0, 127}, thickness = 0.5));
          connect(park.y[2], pi.u) annotation(
            Line(points = {{-27, 0}, {-12, 0}}, color = {0, 0, 127}));
          connect(park.y, Vqds) annotation(
            Line(points = {{-27, 0}, {-20, 0}, {-20, 30}, {70, 30}}, color = {0, 0, 127}, thickness = 0.5));
          connect(pi.y, integrator.u) annotation(
            Line(points = {{12, 0}, {24, 0}}, color = {0, 0, 127}));
          connect(integrator.y, Delta) annotation(
            Line(points = {{48, 0}, {70, 0}}, color = {0, 0, 127}));
          connect(integrator.y, park.theta) annotation(
            Line(points = {{48, 0}, {54, 0}, {54, -22}, {-42, -22}, {-42, -16}}, color = {0, 0, 127}));
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(textColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "PLL")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
            experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
            Diagram);
        end PLL;
      end Funcoes;

      model CONTROL
      import SI = Modelica.Units.SI;
        parameter Real Wrm_min = 0.8052 annotation(
          Dialog(group = "Reference Speed"));
        parameter Real Wrm_nom = 1.115 annotation(
          Dialog(group = "Reference Speed"));
        parameter Real Vw_min = 4 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_nom = 12 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_max = 25 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_wmin = 8.0714 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_wmax = 11.1757 annotation(
          Dialog(group = "Turbine Data"));
        parameter String tableNameR2 annotation(
          Dialog(group = "Lookup table Data"));
        parameter String fileNameR2 annotation(
          Dialog(group = "Lookup table Data"));
        parameter String tableNameR4 annotation(
          Dialog(group = "Lookup table Data"));
        parameter String fileNameR4 annotation(
          Dialog(group = "Lookup table Data"));
        parameter Units.PerUnit kiWrm = 6.04672 "ki by speed" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpWrm = 15.1168 "kp by speed" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiQs = 1 "ki by reactive power" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiPLL = 200 "ki by PLL" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpPLL = 100 "kp by PLL" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIqr = 113135.483871 "ki by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIqr = 282.82871 "kp by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIdr = 113135.483871 "ki by Idr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIdr = 282.82871 "kp by Idr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit Vccref = 1 "Reference voltage by Vcc" annotation(
          Dialog(group = "Reference setings"));
        parameter Units.PerUnit Idgref = 0 "Reference current by Idg" annotation(
          Dialog(group = "Reference setings"));
        parameter Units.PerUnit Qgref = 0 "Reference reactive power" annotation(
          Dialog(group = "Reference setings"));
        parameter Units.PerUnit kiVcc = 2297.408 "ki by Vcc" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpVcc = 574.352 "kp by Vcc" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIqg = 25600.0 "ki by Iqg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIqg = 64.0 "kp by Iqg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIdg = 25600.0 "ki by Idg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIdg = 64.0 "kp by Idg" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit Ls = 3.1 "Stator leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lr = 3.08 "Rotor leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lm = 3.0 "Stator magnetizing inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lc = 0.04 "Filter inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter DWT.Units.PerUnit Ceq = 35.897 "Capacitor of conversor" annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Kc = 0.5/sqrt(2)*1400/(690/sqrt(3)) annotation(
          Dialog(group = "Eletrical Data"));
        //
        DWT.WindTurbine.ControlModel.ControlTurbine controlTurbine(Vw_max = Vw_max, Vw_min = Vw_min, Vw_nom = Vw_nom, Vw_wmax = Vw_wmax, Vw_wmin = Vw_wmin, Wrm_min = Wrm_min, Wrm_nom = Wrm_nom, fileNameR2 = fileNameR2, fileNameR4 = fileNameR4, tableNameR2 = tableNameR2, tableNameR4 = tableNameR4) annotation(
          Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vw annotation(
          Placement(visible = true, transformation(origin = {-124, -30}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-105, -15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Beta annotation(
          Placement(visible = true, transformation(origin = {-65, 1}, extent = {{-7, -7}, {7, 7}}, rotation = 90), iconTransformation(origin = {-105, -1}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));
        DWT.WindTurbine.ControlModel.ReferenceControl referenceControl(Idgref = Idgref, Lm = Lm, Lr = Lr, Ls = Ls, Qgref = Qgref, Vccref = Vccref, kiQs = kiQs, kiVcc = kiVcc, kiWrm = kiWrm, kpVcc = kpVcc, kpWrm = kpWrm) annotation(
          Placement(visible = true, transformation(origin = {-30, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Wmed annotation(
          Placement(visible = true, transformation(origin = {-18, 4}, extent = {{8, -8}, {-8, 8}}, rotation = 90), iconTransformation(origin = {-105, 13}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Qmed annotation(
          Placement(visible = true, transformation(origin = {-42, 4}, extent = {{8, -8}, {-8, 8}}, rotation = 90), iconTransformation(origin = {105, 1}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
          Placement(visible = true, transformation(origin = {-30, 4}, extent = {{8, -8}, {-8, 8}}, rotation = 90), iconTransformation(origin = {53, 31}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput Vt[2] annotation(
          Placement(visible = true, transformation(origin = {-34, 22}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {105, -13}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mr annotation(
          Placement(visible = true, transformation(origin = {134, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {31, 31}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mg annotation(
          Placement(visible = true, transformation(origin = {134, -40}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {77, 31}, extent = {{-5, -5}, {5, 5}}, rotation = 90)));
        DWT.WindTurbine.ControlModel.ModulationControl modulationControl(Kc = Kc) annotation(
          Placement(visible = true, transformation(origin = {90, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Irmed[2] annotation(
          Placement(visible = true, transformation(origin = {-34, 46}, extent = {{8, 8}, {-8, -8}}, rotation = -180), iconTransformation(origin = {-61, 31}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput Igmed[2] annotation(
          Placement(visible = true, transformation(origin = {-34, 34}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {105, 15}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));
        DWT.WindTurbine.ControlModel.CurrentControl currentControl(Lc = Lc, Lm = Lm, Lr = Lr, Ls = Ls, kiIdg = kiIdg, kiIdr = kiIdr, kiIqg = kiIqg, kiIqr = kiIqr, kpIdg = kpIdg, kpIdr = kpIdr, kpIqg = kpIqg, kpIqr = kpIqr) annotation(
          Placement(visible = true, transformation(origin = {30, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.EstimadorControl estimadorControl(kiPLL = kiPLL, kpPLL = kpPLL) annotation(
          Placement(visible = true, transformation(origin = {0, 34}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      equation
      // Conexão textual:
        estimadorControl.Iqdrmed = currentControl.Iqdrmed;
        estimadorControl.Iqdgmed = currentControl.Iqdgmed;
        Vccmed = modulationControl.Vccmed;
        estimadorControl.theta = modulationControl.theta;
      // Conexão explícita:
        connect(Vw, controlTurbine.Vw) annotation(
          Line(points = {{-124, -30}, {-112, -30}}, color = {0, 0, 127}));
        connect(controlTurbine.Wrm_opt, referenceControl.Wref) annotation(
          Line(points = {{-68, -40}, {-52, -40}}, color = {0, 0, 127}));
        connect(controlTurbine.beta, Beta) annotation(
          Line(points = {{-68, -20}, {-65, -20}, {-65, 1}}, color = {0, 0, 127}));
        connect(Vccmed, referenceControl.Vccmed) annotation(
          Line(points = {{-30, 4}, {-30, -8}}, color = {0, 0, 127}));
        connect(Wmed, referenceControl.Wmed) annotation(
          Line(points = {{-18, 4}, {-18, -8}}, color = {0, 0, 127}));
        connect(Qmed, referenceControl.Qmed) annotation(
          Line(points = {{-42, 4}, {-42, -8}}, color = {0, 0, 127}));
        connect(modulationControl.Mqdr, Mr) annotation(
          Line(points = {{112, -20}, {134, -20}}, color = {85, 170, 255}));
        connect(modulationControl.Mqdg, Mg) annotation(
          Line(points = {{112, -40}, {134, -40}}, color = {85, 170, 255}));
        connect(currentControl.Vr, modulationControl.Vqdr) annotation(
          Line(points = {{52, -20}, {68, -20}}, color = {0, 0, 127}, thickness = 0.5));
        connect(currentControl.Vg, modulationControl.Vqdg) annotation(
          Line(points = {{52, -40}, {68, -40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(referenceControl.Iqdr, currentControl.Ir) annotation(
          Line(points = {{-8, -20}, {8, -20}}, color = {0, 0, 127}, thickness = 0.5));
        connect(referenceControl.Iqdg, currentControl.Ig) annotation(
          Line(points = {{-8, -40}, {8, -40}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Irmed, estimadorControl.Irmed) annotation(
          Line(points = {{-34, 46}, {-22, 46}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Vt, estimadorControl.Vt) annotation(
          Line(points = {{-34, 22}, {-22, 22}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Igmed, estimadorControl.Igmed) annotation(
          Line(points = {{-34, 34}, {-22, 34}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Diagram(coordinateSystem(extent = {{-100, -25}, {100, 25}}), graphics = {Text(origin = {89, 3}, rotation = -90, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "Vccmed"), Text(origin = {29, -65}, rotation = -90, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "Iqdrmed"), Text(origin = {39, -65}, rotation = -90, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "Iqdgmed"), Text(origin = {89, -63}, rotation = -90, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "theta"), Text(origin = {-1, -16}, rotation = 180, textColor = {0, 0, 127}, extent = {{-9, 2}, {9, -2}}, textString = "Iqdr"), Text(origin = {-1, -36}, rotation = 180, textColor = {0, 0, 127}, extent = {{-9, 2}, {9, -2}}, textString = "Iqdg"), Text(origin = {59, -36}, rotation = 180, textColor = {0, 0, 127}, extent = {{-9, 2}, {9, -2}}, textString = "Vqdg"), Text(origin = {59, -16}, rotation = 180, textColor = {0, 0, 127}, extent = {{-9, 2}, {9, -2}}, textString = "Vqdr"), Text(origin = {-59, -36}, rotation = 180, textColor = {0, 0, 127}, extent = {{-9, 2}, {9, -2}}, textString = "Wref"), Text(origin = {121, -16}, rotation = 180, textColor = {0, 0, 127}, extent = {{-9, 2}, {9, -2}}, textString = "mqdr"), Text(origin = {121, -36}, rotation = 180, textColor = {0, 0, 127}, extent = {{-9, 2}, {9, -2}}, textString = "mqdg"), Text(origin = {35, 47}, rotation = 180, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "Vqsmed"), Text(origin = {35, 31}, rotation = 180, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "Iqdrmed"), Text(origin = {35, 23}, rotation = 180, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "Iqdgmed"), Text(origin = {33, 39}, rotation = 180, textColor = {0, 0, 127}, extent = {{-12, 2}, {12, -2}}, textString = "theta")}),
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 26}, {100, -26}}), Text(origin = {0, 1}, textColor = {0, 0, 255}, extent = {{-100, 11}, {100, -11}}, textString = "CONTROLE")}, coordinateSystem(extent = {{-100, -25}, {100, 25}})));
      end CONTROL;

      model ControlTurbine
        import pi = Modelica.Constants.pi;
        Modelica.Blocks.Interfaces.RealInput Vw annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-112, 0}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput beta annotation(
          Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Wrm_opt annotation(
          Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter Real Wrm_min = 0.8052 annotation(
          Dialog(group = "Reference Speed"));
        parameter Real Wrm_nom = 1.115 annotation(
          Dialog(group = "Reference Speed"));
        parameter Real Vw_min = 4 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_nom = 12 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_max = 25 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_wmin = 8.0714 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real Vw_wmax = 11.1757 annotation(
          Dialog(group = "Turbine Data"));
        parameter String tableNameR2 annotation(
          Dialog(group = "Lookup table Data"));
        parameter String fileNameR2 annotation(
          Dialog(group = "Lookup table Data"));
        parameter String tableNameR4 annotation(
          Dialog(group = "Lookup table Data"));
        parameter String fileNameR4 annotation(
          Dialog(group = "Lookup table Data"));
        Modelica.Blocks.Tables.CombiTable1Ds lookupTable_Beta(fileName = fileNameR4, tableName = tableNameR4, tableOnFile = true, verboseRead = false) annotation(
          Placement(visible = true, transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Tables.CombiTable1Ds lookupTable_Wrm(fileName = fileNameR2, tableName = tableNameR2, tableOnFile = true, verboseRead = false) annotation(
          Placement(visible = true, transformation(origin = {-50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      algorithm
        if Vw >= Vw_min and Vw < Vw_wmin then
          Wrm_opt := Wrm_min;
          beta := 0;
        elseif Vw >= Vw_wmin and Vw < Vw_wmax then
          lookupTable_Wrm.u := Vw - Vw_wmin;
          Wrm_opt := lookupTable_Wrm.y[1];
          beta := 0;
        elseif Vw >= Vw_wmax and Vw < Vw_nom then
          Wrm_opt := Wrm_nom;
          beta := 0;
        elseif Vw >= Vw_nom and Vw <= Vw_max then
          Wrm_opt := Wrm_nom;
          lookupTable_Beta.u := Vw - Vw_nom;
          beta := lookupTable_Beta.y[1];
        end if;
      equation

        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-20, 79}, textColor = {0, 0, 255}, extent = {{-80, 19}, {80, -19}}, textString = "WIND"), Line(points = {{-100, -100}, {100, 100}, {100, 100}}, color = {0, 0, 255}), Text(origin = {21, -79}, textColor = {0, 0, 255}, extent = {{-79, 21}, {79, -21}}, textString = "REF")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
          experiment(StartTime = 0, StopTime = 510, Tolerance = 1e-06, Interval = 0.005));
      end ControlTurbine;

      model EstimadorControl
        parameter Units.PerUnit kiPLL = 200 "ki by PLL" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpPLL = 100 "kp by PLL" annotation(
          Dialog(group = "Control setings"));
        Modelica.Blocks.Interfaces.RealInput Irmed[2] annotation(
          Placement(visible = true, transformation(origin = {22, 20}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vt[2] annotation(
          Placement(visible = true, transformation(origin = {-104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.T tg annotation(
          Placement(visible = true, transformation(origin = {50, -20}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Igmed[2] annotation(
          Placement(visible = true, transformation(origin = {22, -20}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.PLL pll(ki = kiPLL, kp = kpPLL) annotation(
          Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.T tr annotation(
          Placement(visible = true, transformation(origin = {50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Iqdrmed[2] annotation(
          Placement(visible = true, transformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Iqdgmed[2] annotation(
          Placement(visible = true, transformation(origin = {110, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput theta annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(pll.Delta, tr.theta) annotation(
          Line(points = {{-9, 0}, {-1, 0}, {-1, 9}, {50, 9}}, color = {0, 0, 127}));
        connect(Vt, pll.Vt) annotation(
          Line(points = {{-104, 0}, {-31, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(pll.Delta, tg.theta) annotation(
          Line(points = {{-9, 0}, {-1, 0}, {-1, -9}, {50, -9}}, color = {0, 0, 127}));
        connect(tr.y, Iqdrmed) annotation(
          Line(points = {{61, 20}, {110, 20}}, color = {0, 0, 127}, thickness = 0.5));
        connect(tg.y, Iqdgmed) annotation(
          Line(points = {{61, -20}, {110, -20}}, color = {0, 0, 127}, thickness = 0.5));
        connect(pll.Delta, theta) annotation(
          Line(points = {{-8, 0}, {110, 0}}, color = {0, 0, 127}));
        connect(Igmed, tg.u) annotation(
          Line(points = {{22, -20}, {38, -20}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Irmed, tr.u) annotation(
          Line(points = {{22, 20}, {38, 20}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(origin = {-1, 0}, points = {{-99, -100}, {101, 100}, {101, 100}}), Text(origin = {-20, 80}, textColor = {0, 0, 255}, extent = {{-80, 20}, {80, -20}}, textString = "SYS"), Text(origin = {20, -80}, textColor = {0, 0, 255}, extent = {{-80, 20}, {80, -20}}, textString = "MED")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end EstimadorControl;

      model ReferenceControl
        import SI = Modelica.Units.SI;
        parameter Units.PerUnit kiWrm = 6.04672 "ki by speed" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpWrm = 15.1168 "kp by speed" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiQs = 1 "ki by reactive power" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit Vccref = 1 "Reference voltage by Vcc" annotation(
          Dialog(group = "Reference setings"));
        parameter Units.PerUnit Idgref = 0 "Reference current by Idg" annotation(
          Dialog(group = "Reference setings"));
        parameter Units.PerUnit Qgref = 0 "Reference reactive power" annotation(
          Dialog(group = "Reference setings"));
        parameter Units.PerUnit kiVcc = 2297.408 "ki by Vcc" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpVcc = 574.352 "kp by Vcc" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit Ls = 3.1 "Stator leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lr = 3.08 "Rotor leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lm = 3.0 "Stator magnetizing inductance" annotation(
          Dialog(group = "Electrical Data"));
        //
        Modelica.Blocks.Interfaces.RealInput Wmed annotation(
          Placement(visible = true, transformation(origin = {-90, 56}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Sources.Constant refIdg(k = Idgref) annotation(
          Placement(visible = true, transformation(origin = {10, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
          Placement(visible = true, transformation(origin = {-90, -56}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regW(ki = -kiWrm, kp = -kpWrm) annotation(
          Placement(visible = true, transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.Limiter Limiter3(maxMod = 1) annotation(
          Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.Limiter Limiter1(maxMod = 1) annotation(
          Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product quad1 annotation(
          Placement(visible = true, transformation(origin = {-16, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant fQgref(k = Qgref) annotation(
          Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regQ(ki = kiQs, kp = 0) annotation(
          Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM pi_astrom3(ki = kiVcc, kp = kpVcc) annotation(
          Placement(visible = true, transformation(origin = {10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product quad2 annotation(
          Placement(visible = true, transformation(origin = {-16, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Qmed annotation(
          Placement(visible = true, transformation(origin = {-90, 6}, extent = {{8, 8}, {-8, -8}}, rotation = 180), iconTransformation(origin = {-60, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Sources.Constant fVccref(k = Vccref) annotation(
          Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Wref annotation(
          Placement(visible = true, transformation(origin = {-90, 70}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Iqdr[2] annotation(
          Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Iqdg[2] annotation(
          Placement(visible = true, transformation(origin = {90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(pi_astrom3.y, Limiter3.u[1]) annotation(
          Line(points = {{21.2, -30}, {39, -30}, {39, -50}}, color = {0, 0, 127}));
        connect(fVccref.y, quad1.u2) annotation(
          Line(points = {{-79, -30}, {-34, -30}, {-34, -36}, {-28, -36}}, color = {0, 0, 127}));
        connect(Wref, regW.r) annotation(
          Line(points = {{-90, 70}, {-1, 70}}, color = {0, 0, 127}));
        connect(Wmed, regW.m) annotation(
          Line(points = {{-90, 56}, {-90, 59}, {10, 59}}, color = {0, 0, 127}));
        connect(Vccmed, quad2.u1) annotation(
          Line(points = {{-90, -56}, {-36, -56}, {-36, -50}, {-28, -50}}, color = {0, 0, 127}));
        connect(Vccmed, quad2.u2) annotation(
          Line(points = {{-90, -56}, {-36, -56}, {-36, -62}, {-28, -62}}, color = {0, 0, 127}));
        connect(Qmed, regQ.m) annotation(
          Line(points = {{-90, 6}, {-16, 6}, {-16, 19}, {10, 19}}, color = {0, 0, 127}));
        connect(fQgref.y, regQ.r) annotation(
          Line(points = {{-79, 30}, {-1, 30}}, color = {0, 0, 127}));
        connect(quad2.y, pi_astrom3.m) annotation(
          Line(points = {{-5, -56}, {10, -56}, {10, -41}}, color = {0, 0, 127}));
        connect(refIdg.y, Limiter3.u[2]) annotation(
          Line(points = {{21, -80}, {39, -80}, {39, -50}}, color = {0, 0, 127}));
        connect(quad1.y, pi_astrom3.r) annotation(
          Line(points = {{-5, -30}, {-2, -30}}, color = {0, 0, 127}));
        connect(fVccref.y, quad1.u1) annotation(
          Line(points = {{-79, -30}, {-34, -30}, {-34, -24}, {-28, -24}}, color = {0, 0, 127}));
        connect(Limiter1.y, Iqdr) annotation(
          Line(points = {{61, 50}, {90, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Limiter3.y, Iqdg) annotation(
          Line(points = {{60, -50}, {90, -50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(regW.y, Limiter1.u[1]) annotation(
          Line(points = {{21, 70}, {40, 70}, {40, 50}}, color = {0, 0, 127}));
  connect(regQ.y, Limiter1.u[2]) annotation(
          Line(points = {{21, 30}, {40, 30}, {40, 50}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}, color = {0, 0, 255}), Text(origin = {-21, 80}, textColor = {0, 0, 255}, extent = {{-79, 20}, {79, -20}}, textString = "REF"), Text(origin = {19, -81}, textColor = {0, 0, 255}, extent = {{81, -19}, {-81, 19}}, textString = "CUR")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end ReferenceControl;

      model CurrentControl
        import SI = Modelica.Units.SI;
        parameter DWT.Units.PerUnit kiIqr = 113135.483871 "ki by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit kpIqr = 282.82871 "kp by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit kiIdr = 113135.483871 "ki by Idr" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit kpIdr = 282.82871 "kp by Idr" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit kiIqg = 25600.0 "ki by Iqg" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit kpIqg = 64.0 "kp by Iqg" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit kiIdg = 25600.0 "ki by Idg" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit kpIdg = 64.0 "kp by Idg" annotation(
          Dialog(group = "Control setings"));
        parameter DWT.Units.PerUnit Ls = 3.1 "Stator leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter DWT.Units.PerUnit Lr = 3.08 "Rotor leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter DWT.Units.PerUnit Lm = 3.0 "Stator magnetizing inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter DWT.Units.PerUnit Lc = 0.04 "Filter inductance" annotation(
          Dialog(group = "Eletrical Data"));
        //
        Modelica.Blocks.Interfaces.RealInput Ig[2] annotation(
          Placement(visible = true, transformation(origin = {-38, -50}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Iqdgmed[2] annotation(
          Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealInput Iqdrmed[2] annotation(
          Placement(visible = true, transformation(origin = {-120, 52}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Funcoes.Limiter Limiter4 annotation(
          Placement(visible = true, transformation(origin = {60, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Funcoes.Limiter Limiter2 annotation(
          Placement(visible = true, transformation(origin = {60, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Funcoes.PI_ASTROM pi_astrom3(ki = kiIdg, kp = kpIdg) annotation(
          Placement(visible = true, transformation(origin = {-10, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Funcoes.PI_ASTROM pi_astrom(ki = kiIdr, kp = kpIdr) annotation(
          Placement(visible = true, transformation(origin = {-10, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        Funcoes.PI_ASTROM pi_astrom1(ki = kiIqr, kp = kpIqr) annotation(
          Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Funcoes.PI_ASTROM pi_astrom2(ki = kiIqg, kp = kpIqg) annotation(
          Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Ir[2] annotation(
          Placement(visible = true, transformation(origin = {-38, 52}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Vr[2] annotation(
          Placement(visible = true, transformation(origin = {108, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Vg[2] annotation(
          Placement(visible = true, transformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(Ig[1], pi_astrom2.r) annotation(
          Line(points = {{-38, -50}, {-28, -50}, {-28, -30}, {-22, -30}}, color = {0, 0, 127}));
        connect(Iqdgmed[2], pi_astrom3.m) annotation(
          Line(points = {{-120, -50}, {-48, -50}, {-48, -82}, {-10, -82}}, color = {0, 0, 127}));
        connect(Iqdgmed[1], pi_astrom2.m) annotation(
          Line(points = {{-120, -50}, {-48, -50}, {-48, -18}, {-10, -18}}, color = {0, 0, 127}));
        connect(Ig[2], pi_astrom3.r) annotation(
          Line(points = {{-38, -50}, {-30, -50}, {-30, -70}, {-22, -70}}, color = {0, 0, 127}));
        connect(Ir[2], pi_astrom.r) annotation(
          Line(points = {{-38, 52}, {-28, 52}, {-28, 70}, {-22, 70}}, color = {0, 0, 127}));
        connect(Ir[1], pi_astrom1.r) annotation(
          Line(points = {{-38, 52}, {-28, 52}, {-28, 30}, {-22, 30}}, color = {0, 0, 127}));
        connect(Iqdrmed[2], pi_astrom.m) annotation(
          Line(points = {{-120, 52}, {-60, 52}, {-60, 82}, {-10, 82}}, color = {0, 0, 127}));
        connect(Iqdrmed[1], pi_astrom1.m) annotation(
          Line(points = {{-120, 52}, {-60, 52}, {-60, 18}, {-10, 18}}, color = {0, 0, 127}));
        connect(Limiter2.y, Vr) annotation(
          Line(points = {{70, 50}, {108, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Limiter4.y, Vg) annotation(
          Line(points = {{70, -50}, {108, -50}}, color = {0, 0, 127}, thickness = 0.5));
  connect(pi_astrom2.y, Limiter4.u[1]) annotation(
          Line(points = {{2, -30}, {50, -30}, {50, -50}}, color = {0, 0, 127}));
  connect(pi_astrom3.y, Limiter4.u[2]) annotation(
          Line(points = {{2, -70}, {50, -70}, {50, -50}}, color = {0, 0, 127}));
  connect(pi_astrom.y, Limiter2.u[2]) annotation(
          Line(points = {{2, 70}, {50, 70}, {50, 50}}, color = {0, 0, 127}));
  connect(pi_astrom1.y, Limiter2.u[1]) annotation(
          Line(points = {{2, 30}, {50, 30}, {50, 50}}, color = {0, 0, 127}));
        annotation(
          Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})),
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(origin = {-0.53, 0}, points = {{-100, -100}, {100, 100}, {100, 100}}, color = {0, 0, 255}), Text(origin = {-19, 80}, textColor = {0, 0, 255}, extent = {{-81, 20}, {81, -20}}, textString = "CUR"), Text(origin = {20, -81}, textColor = {0, 0, 255}, extent = {{80, -19}, {-80, 19}}, textString = "VOL")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end CurrentControl;

      model ModulationControl
        parameter DWT.Units.PerUnit Kc = 0.5*(sqrt(3)/sqrt(2))*(1400/690);
        DWT.WindTurbine.ControlModel.Funcoes.Modulation modulationg(Kc = Kc) annotation(
          Placement(visible = true, transformation(origin = {-50, -50}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mqdr annotation(
          Placement(visible = true, transformation(origin = {106, 50}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mqdg annotation(
          Placement(visible = true, transformation(origin = {106, -50}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.ComplexBlocks.ComplexMath.RealToComplex real2Complex2 annotation(
          Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.Modulation modulationr(Kc = Kc) annotation(
          Placement(visible = true, transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.iT ipark1 annotation(
          Placement(visible = true, transformation(origin = {0, -50}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
          Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.ComplexBlocks.ComplexMath.RealToComplex real2Complex1 annotation(
          Placement(visible = true, transformation(origin = {50, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput theta annotation(
          Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        DWT.WindTurbine.ControlModel.Funcoes.iT ipark annotation(
          Placement(visible = true, transformation(origin = {0, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vqdr[2] annotation(
          Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vqdg[2] annotation(
          Placement(visible = true, transformation(origin = {-118, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(real2Complex1.y, Mqdr) annotation(
          Line(points = {{61, 50}, {106, 50}}, color = {85, 170, 255}));
        connect(ipark.y[1], real2Complex1.re) annotation(
          Line(points = {{11, 50}, {11, 56}, {38, 56}}, color = {0, 0, 127}));
        connect(ipark1.y[1], real2Complex2.re) annotation(
          Line(points = {{11, -50}, {11, -44}, {38, -44}}, color = {0, 0, 127}));
        connect(ipark.y[2], real2Complex1.im) annotation(
          Line(points = {{11, 50}, {11, 44}, {38, 44}}, color = {0, 0, 127}));
        connect(Vccmed, modulationg.Vccmed) annotation(
          Line(points = {{-60, 0}, {-50, 0}, {-50, -38}}, color = {0, 0, 127}));
        connect(modulationr.y, ipark.u) annotation(
          Line(points = {{-39, 50}, {-11, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(theta, ipark.theta) annotation(
          Line(points = {{-10, 0}, {0, 0}, {0, 40}}, color = {0, 0, 127}));
        connect(modulationg.y, ipark1.u) annotation(
          Line(points = {{-39, -50}, {-11, -50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(real2Complex2.y, Mqdg) annotation(
          Line(points = {{61, -50}, {106, -50}}, color = {85, 170, 255}));
        connect(theta, ipark1.theta) annotation(
          Line(points = {{-10, 0}, {0, 0}, {0, -38}}, color = {0, 0, 127}));
        connect(Vccmed, modulationr.Vccmed) annotation(
          Line(points = {{-60, 0}, {-50, 0}, {-50, 40}}, color = {0, 0, 127}));
        connect(ipark1.y[2], real2Complex2.im) annotation(
          Line(points = {{11, -50}, {11, -56}, {38, -56}}, color = {0, 0, 127}));
        connect(Vqdr, modulationr.u) annotation(
          Line(points = {{-120, 50}, {-61, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Vqdg, modulationg.u) annotation(
          Line(points = {{-118, -50}, {-61, -50}}, color = {0, 0, 127}, thickness = 0.5));
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}, color = {0, 0, 255}), Text(origin = {-20, 80}, textColor = {0, 0, 255}, extent = {{-80, 20}, {80, -20}}, textString = "VOL"), Text(origin = {21, -80}, textColor = {0, 0, 255}, extent = {{81, -20}, {-81, 20}}, textString = "MOD")}));
      end ModulationControl;
    end ControlModel;

    model Sensors
      extends Modelica.Icons.SensorsPackage;

      model CurrentSensor
        extends Modelica.Icons.RoundSensor;
        DWT.Circuit.Interfaces.PositivePin pin_p annotation(
          Placement(visible = true, transformation(origin = {-96, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Interfaces.NegativePin pin_n annotation(
          Placement(visible = true, transformation(origin = {96, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput outI[2] annotation(
          Placement(visible = true, transformation(origin = {0, -74}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
// voltage treatment:
        pin_p.v.re = pin_n.v.re;
        pin_p.v.im = pin_n.v.im;
// current treatment:
        pin_p.i.re = outI[1];
        pin_p.i.im = outI[2];
        pin_n.i.re = -outI[1];
        pin_n.i.im = -outI[2];
        annotation(
          Icon(graphics = {Text(origin = {2, -34}, extent = {{-32, 10}, {32, -10}}, textString = "Current")}));
      end CurrentSensor;

      model VoltageSensor
        extends Modelica.Icons.RoundSensor;
        DWT.Circuit.Interfaces.PositivePin pin_p annotation(
          Placement(visible = true, transformation(origin = {-86, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput outV[2] annotation(
          Placement(visible = true, transformation(origin = {-44, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
// voltage treatment:
        pin_p.v.re = outV[1];
        pin_p.v.im = outV[2];
// current treatment:
        pin_p.i.re = 0;
        pin_p.i.im = 0;
        annotation(
          Icon(graphics = {Text(origin = {1, -34}, extent = {{-29, 14}, {29, -14}}, textString = "V"), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {90, 0}, points = {{-18, 0}, {18, 0}, {18, 32}, {18, -32}, {18, -30}}), Line(origin = {122, -1}, points = {{0, -19}, {0, 19}, {0, 19}}), Line(origin = {134, -1}, points = {{0, -5}, {0, 5}, {0, 5}})}));
      end VoltageSensor;

      model PowerSensor
        extends Modelica.Icons.RoundSensor;
        import Modelica.ComplexMath.conj;
        DWT.Circuit.Interfaces.PositivePin pin_p annotation(
          Placement(visible = true, transformation(origin = {-96, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Interfaces.NegativePin pin_n annotation(
          Placement(visible = true, transformation(origin = {96, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput outS[2] annotation(
          Placement(visible = true, transformation(origin = {0, -74}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Complex S "Complex power";
      equation
// voltage treatment:
        pin_p.v = pin_n.v;
// current treatment:
        pin_p.i = -pin_n.i;
// S:
        S = -pin_p.v*conj(pin_p.i);
        outS[1] = S.re;
        outS[2] = S.im;
        annotation(
          Icon(graphics = {Text(origin = {2, -34}, extent = {{-32, 10}, {32, -10}}, textString = "VA")}));
      end PowerSensor;
    equation

    end Sensors;

    package Examples
      extends Modelica.Icons.ExamplesPackage;

      model Exemplo_MPPT
        extends Modelica.Icons.Example;
        inner DWT.SystemData data(Sbase = 2) annotation(
          Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Sources.VoltageSource voltageSource annotation(
          Placement(visible = true, transformation(origin = {31, 11}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
        Modelica.Blocks.Sources.Ramp ramp(duration = 990, height = 20.9, offset = 4, startTime = 5) annotation(
          Placement(visible = true, transformation(origin = {-32, 16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        DFIG dfig(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter DWT.WindTurbine.Interfaces.DWTData smData(MVAs = data.Sbase, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/betaR4.mat") annotation(
          Placement(visible = true, transformation(origin = {-10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(ramp.y, dfig.Vw) annotation(
          Line(points = {{-25, 16}, {-10, 16}}, color = {0, 0, 127}));
        connect(dfig.pin_DFIG, voltageSource.p) annotation(
          Line(points = {{12, 16}, {31, 16}}, color = {0, 0, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-06, Interval = 0.001));
      end Exemplo_MPPT;

      model Exemplo_PotReativa
        extends Modelica.Icons.Example;
        inner DWT.SystemData data(Sbase = 2) annotation(
          Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Sources.VoltageSource voltageSource annotation(
          Placement(visible = true, transformation(origin = {31, 11}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
        Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = -1, offset = 12, startTime = 5) annotation(
          Placement(visible = true, transformation(origin = {-30, 16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        DFIG dfig(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter DWT.WindTurbine.Interfaces.DWTData smData(Idgref = 0, MVAs = data.Sbase, Qgref = -0.5, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/betaR4.mat") annotation(
          Placement(visible = true, transformation(origin = {-10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(ramp.y, dfig.Vw) annotation(
          Line(points = {{-23, 16}, {-10, 16}}, color = {0, 0, 127}));
        connect(dfig.pin_DFIG, voltageSource.p) annotation(
          Line(points = {{12, 16}, {31, 16}}, color = {0, 0, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.001));
      end Exemplo_PotReativa;

      model Exemplo_AfundTensao
        extends Modelica.Icons.Example;
        inner DWT.SystemData data(Sbase = 2) annotation(
          Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = 0, offset = 12, startTime = 0) annotation(
          Placement(visible = true, transformation(origin = {-30, 16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        DFIG dfig(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter DWT.WindTurbine.Interfaces.DWTData smData(MVAs = data.Sbase, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/betaR4.mat") annotation(
          Placement(visible = true, transformation(origin = {-10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Sources.ControlledVoltageSource controlledVoltageSource annotation(
          Placement(visible = true, transformation(origin = {30, 6}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        if time < 1 then
          controlledVoltageSource.v = Complex(1, 0);
        elseif time > 1 and time <= 1.1 then
          controlledVoltageSource.v = Complex(0.5, 0);
        else
          controlledVoltageSource.v = Complex(1, 0);
        end if;
        connect(ramp.y, dfig.Vw) annotation(
          Line(points = {{-23, 16}, {-10, 16}}, color = {0, 0, 127}));
        connect(dfig.pin_DFIG, controlledVoltageSource.p) annotation(
          Line(points = {{12, 16}, {30, 16}}, color = {0, 0, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.001));
      end Exemplo_AfundTensao;

      model Exemplo_SistemaRadial
        extends Modelica.Icons.Example;
        inner DWT.SystemData data(Sbase = 2) annotation(
          Placement(visible = true, transformation(origin = {10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Sources.VoltageSource voltageSource annotation(
          Placement(visible = true, transformation(origin = {61, 11}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
        Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = -2, offset = 12, startTime = 5) annotation(
          Placement(visible = true, transformation(origin = {-30, 16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        DFIG dfig(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter DWT.WindTurbine.Interfaces.DWTData smData(MVAs = data.Sbase, Qgref = -0.5, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Notebooks Python/LookupTables/betaR4.mat") annotation(
          Placement(visible = true, transformation(origin = {-10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Circuit.Basic.SeriesImpedance seriesImpedance(r = 0, x = 0.01) annotation(
          Placement(visible = true, transformation(origin = {36, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(ramp.y, dfig.Vw) annotation(
          Line(points = {{-23, 16}, {-10, 16}}, color = {0, 0, 127}));
        connect(seriesImpedance.n, voltageSource.p) annotation(
          Line(points = {{46, 16}, {62, 16}}, color = {0, 0, 255}));
        connect(dfig.pin_DFIG, seriesImpedance.p) annotation(
          Line(points = {{12, 16}, {26, 16}}, color = {0, 0, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.001),
          __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
          __OpenModelica_simulationFlags(lv = "LOG_STATS", s = "cvode"));
      end Exemplo_SistemaRadial;
    end Examples;
  end WindTurbine;
  annotation(
    uses(Modelica(version = "4.0.0")));
end DWT;