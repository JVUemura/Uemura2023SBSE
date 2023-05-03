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
        Placement(visible = true, transformation(origin = {0, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.WindTurbine.TurbineModel.EOLICA eolica(Dtm = smData.convData.Dtm,Ht = smData.convData.Ht, Ktm = smData.convData.Ktm, N = smData.N, Pb = 1e6*smData.MVAb, R = smData.R, Wrmb = smData.Wrmb, kb = smData.kb, par = smData.par, tb = smData.tb) annotation(
        Placement(visible = true, transformation(origin = {-32, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.WindTurbine.ConversorModel.CONVERSOR conversor(Ceq = smData.convData.Ceq) annotation(
        Placement(visible = true, transformation(origin = {30, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.WindTurbine.Sensors.CurrentSensor currentSensor annotation(
        Placement(visible = true, transformation(origin = {10, -6}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      DWT.WindTurbine.Sensors.CurrentSensor currentSensor1 annotation(
        Placement(visible = true, transformation(origin = {50, -6}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      DWT.WindTurbine.ControlModel.ControlMachine ControlMachine(Ceq = smData.convData.Ceq, Idgref = smData.Idgref, Kc = smData.Kc, Lm = smData.convData.Lm, Lr = smData.convData.Llr + smData.convData.Lm, Ls = smData.convData.Lls + smData.convData.Lm, Qgref = smData.Qgref, Vccref = smData.Vccref, kiIdg = smData.kiIdg, kiIdr = smData.kiIdr, kiIqg = smData.kiIqg, kiIqr = smData.kiIqr, kiPLL = smData.kiPLL, kiQs = smData.kiQs, kiVcc = smData.kiVcc, kiWrm = smData.kiWrm, kpIdg = smData.kpIdg, kpIdr = smData.kpIdr, kpIqg = smData.kpIqg, kpIqr = smData.kpIqr, kpPLL = smData.kpPLL, kpVcc = smData.kpVcc, kpWrm = smData.kpWrm) annotation(
        Placement(visible = true, transformation(origin = {30, -32}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      DWT.WindTurbine.Sensors.PowerSensor powerSensor annotation(
        Placement(visible = true, transformation(origin = {58, 14}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      DWT.WindTurbine.Sensors.VoltageSensor voltageSensor annotation(
        Placement(visible = true, transformation(origin = {65, -31}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor tac annotation(
        Placement(visible = true, transformation(origin = {-1, -27}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      DWT.WindTurbine.ControlModel.ControlTurbine ControlTurbine(Vw_max = smData.Vw_max, Vw_min = smData.Vw_min, Vw_nom = smData.Vw_nom, Vw_wmax = smData.Vw_wmax, Vw_wmin = smData.Vw_wmin, Wrm_min = smData.Wrm_min, Wrm_nom = smData.Wrm_nom, fileNameR2 = smData.fileNameR2, fileNameR4 = smData.fileNameR4, tableNameR2 = smData.tableNameR2, tableNameR4 = smData.tableNameR4) annotation(
        Placement(visible = true, transformation(origin = {-29, -35}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-48, 14}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      DWT.Circuit.Interfaces.PositivePin pin_DFIG annotation(
        Placement(visible = true, transformation(origin = {66, 14}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(currentSensor.pin_p, conversor.outR) annotation(
        Line(points = {{14, -6}, {18.6, -6}}));
      connect(conversor.outG, currentSensor1.pin_p) annotation(
        Line(points = {{41.4, -6}, {46, -6}}));
      connect(conversor.outVcc, ControlMachine.Vccmed) annotation(
        Line(points = {{30, -13.4}, {30, -21}}, color = {0, 0, 127}));
      connect(conversor.Mr, ControlMachine.Mqdr) annotation(
        Line(points = {{24, -15.2}, {24, -21}}, color = {85, 170, 255}));
      connect(conversor.Mg, ControlMachine.Mqdg) annotation(
        Line(points = {{36, -15.2}, {36, -21}}, color = {85, 170, 255}));
      connect(currentSensor.outI, ControlMachine.Iqdr) annotation(
        Line(points = {{10, -10}, {10, -23.6}, {17, -23.6}}, color = {0, 0, 127}, thickness = 0.5));
      connect(mit.pin_estator, powerSensor.pin_p) annotation(
        Line(points = {{11, 14}, {54, 14}}));
      connect(ControlMachine.Vqds, voltageSensor.outV) annotation(
        Line(points = {{43, -31}, {61, -31}}, color = {0, 0, 127}, thickness = 0.5));
      connect(tac.w, ControlMachine.Wmed) annotation(
        Line(points = {{4.5, -27}, {17, -27}}, color = {0, 0, 127}));
      connect(eolica.Beta, ControlTurbine.beta) annotation(
        Line(points = {{-29.2, 4}, {-29, 4}, {-29, -25}}, color = {0, 0, 127}));
      connect(Vw, eolica.Vw) annotation(
        Line(points = {{-48, 14}, {-36, 14}}, color = {0, 0, 127}));
      connect(pin_DFIG, powerSensor.pin_n) annotation(
        Line(points = {{66, 14}, {62, 14}}, color = {0, 0, 255}));
  connect(eolica.flange_Eixo, mit.eixo) annotation(
        Line(points = {{-26, 14}, {0, 14}}));
  connect(tac.flange, mit.eixo) annotation(
        Line(points = {{-6, -27}, {-6, 14}, {0, 14}}));
  connect(currentSensor.pin_n, mit.pin_rotor) annotation(
        Line(points = {{6, -6}, {0, -6}, {0, 8}}, color = {0, 0, 255}));
  connect(ControlMachine.Iqdg, currentSensor1.outI) annotation(
        Line(points = {{44, -24}, {50, -24}, {50, -10}}, color = {0, 0, 127}, thickness = 0.5));
  connect(ControlMachine.Qmed, powerSensor.outS[2]) annotation(
        Line(points = {{44, -28}, {58, -28}, {58, 10}}, color = {0, 0, 127}));
  connect(pin_DFIG, voltageSensor.pin_p) annotation(
        Line(points = {{66, 14}, {65, 14}, {65, -26}}, color = {0, 0, 255}));
  connect(Vw, ControlTurbine.Vw) annotation(
        Line(points = {{-48, 14}, {-48, -31}, {-41, -31}}, color = {0, 0, 127}));
  connect(currentSensor1.pin_n, powerSensor.pin_p) annotation(
        Line(points = {{54, -6}, {54, 14}}, color = {0, 0, 255}));
  connect(ControlTurbine.Wrm_opt, ControlMachine.Wref) annotation(
        Line(points = {{-16, -30}, {16, -30}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.001),
        Diagram(graphics = {Rectangle(origin = {-31, 14}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-9, 22}, {9, -22}}), Rectangle(origin = {-29, -29}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-17, 9}, {17, -9}}), Rectangle(origin = {1, 13}, fillColor = {255, 255, 255}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-13, 13}, {13, -13}}), Rectangle(origin = {30, -29}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-18, 9}, {18, -9}}), Rectangle(origin = {30, -6}, pattern = LinePattern.Dot, lineThickness = 0.5, extent = {{-14, 10}, {14, -10}}), Text(origin = {0, 32}, extent = {{-12, 4}, {12, -4}}, textString = "MODELO
MIT"), Text(origin = {-31, 42}, extent = {{-17, 4}, {17, -4}}, textString = "MODELO
TURBINA"), Text(origin = {-29, -44}, extent = {{-17, 4}, {17, -4}}, textString = "CONTROLE
    MECÂNICO"), Text(origin = {31, -44}, extent = {{-17, 4}, {17, -4}}, textString = "CONTROLE
    ELÉTRICO"), Text(origin = {31, 8}, extent = {{-15, 4}, {15, -4}}, textString = "MODELO
CONVERSOR")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
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
        parameter Units.PerUnit Kc = 0.5/sqrt(2)*1400/(690/sqrt(3)) "Modulation base" annotation(
          Dialog(group = "Conversor Data"));
        parameter Units.PerUnit Ceq = 35.897 "Capacitor of converter" annotation(
          Dialog(group = "Conversor Data"));
        // Control pitch data:
        parameter Units.PerUnit tb = 0.0125 "Controler pitch angle" annotation(
          Dialog(group = "Pitch Control Data"));
        parameter Units.PerUnit kb = 20 "Controler by pitch angle" annotation(
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

        ConvertedData convData(Rs = MVAs/MVAb/Nmaq*Rs, Rr = MVAs/MVAb/Nmaq*Rr, Lls = MVAs/MVAb/Nmaq*Lls, Llr = MVAs/MVAb/Nmaq*Llr, Lm = MVAs/MVAb/Nmaq*Lm, Hm = Nmaq*MVAb/MVAs*Hm, Ht = Nmaq*MVAb/MVAs*Ht, Ktm = Nmaq*MVAb/MVAs*Ktm, Dtm = Nmaq*MVAb/MVAs*Dtm, Ceq = MVAs/MVAb/Nmaq*Ceq);
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

      model EOLICA
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
        parameter Real kb = 20 annotation(
          Dialog(group = "Control Calculed"));
        parameter Real tb = 0.0125 annotation(
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
        Modelica.Blocks.Continuous.TransferFunction tfBeta(a = {tb, 1, kb}, b = {kb}, initType =    Modelica.Blocks.Types.Init.SteadyState) annotation(
          Placement(visible = true, transformation(origin = {-74, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Components.Inertia inertia_tur(J = 2*Ht) annotation(
          Placement(visible = true, transformation(origin = {32, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
          Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.SpringDamper eixo(a_rel(fixed = false), c = Ktm, d = Dtm, phi_rel(fixed = false), phi_rel0(displayUnit = "rad"), w_rel(fixed = false)) annotation(
          Placement(visible = true, transformation(origin = {68, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
        connect(Beta, tfBeta.u) annotation(
          Line(points = {{-118, -58}, {-86, -58}}, color = {0, 0, 127}));
        connect(torque.flange, inertia_tur.flange_b) annotation(
          Line(points = {{8, 0}, {22, 0}}));
  connect(inertia_tur.flange_a, eixo.flange_a) annotation(
          Line(points = {{42, 0}, {58, 0}}));
  connect(eixo.flange_b, flange_Eixo) annotation(
          Line(points = {{78, 0}, {104, 0}}));
        annotation(
          Icon(graphics = {Rectangle(origin = {35, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-15, 6}, {15, -6}}), Rectangle(origin = {3, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-17, 20}, {17, -20}}), Ellipse(origin = {-10, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-20, 20}, {20, -20}}), Polygon(origin = {0, 80}, lineColor = {0, 0, 255}, points = {{6, -60}, {0, -60}, {0, 120}, {20, 0}, {20, 0}, {6, -60}}), Polygon(origin = {0, -110}, lineColor = {0, 0, 255}, points = {{0, 90}, {0, -90}, {20, 28}, {6, 90}, {0, 90}, {0, 90}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
      end EOLICA;
    end TurbineModel;

    package MachineModel
      extends Modelica.Icons.Package;

      model MIT
        import SI = Modelica.Units.SI;
        //  Declarando variáveis do problema:
        DWT.Units.PerUnit Wrm, Te;
        DWT.Units.PerUnit Vqs, Vds, Vqr, Vdr;
        DWT.Units.PerUnit Iqs, Ids, Iqr, Idr;
        DWT.Units.PerUnit fqs, fds, fqr, fdr;
        //  Parâmetros MIT (2 MW, 690V, 60Hz):
        parameter DWT.Units.PerUnit Rs = 0.01 annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Rr = 0.01 annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Lls = 0.1 annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Llr = 0.08 annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Lm = 3.0 annotation(
          Dialog(group = "Eletrical Data"));
        parameter SI.Time Hm = 0.52 annotation(
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
        parameter DWT.Units.PerUnit Kc = 1400/(690/sqrt(3))/2/sqrt(2) annotation(
          Dialog(group = "Eletrical Data"));
        parameter Units.PerUnit Ceq = 35.897 "Capacitor of conversor" annotation(
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
          Placement(visible = true, transformation(origin = {88, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {114, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
          Line(points = {{70, 20}, {88, 20}}, color = {0, 0, 255}));
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
            Placement(visible = true, transformation(origin = {32, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
            Placement(visible = true, transformation(origin = {-98, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
          Modelica.Blocks.Interfaces.RealInput theta annotation(
            Placement(visible = true, transformation(origin = {-22, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        equation
          y[1] = cos(theta)*u[1] + sin(theta)*u[2];
          y[2] = (-sin(theta)*u[1]) + cos(theta)*u[2];
          annotation(
            Diagram,
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}, color = {0, 0, 255}), Text(origin = {-26, 69}, lineColor = {0, 0, 255}, extent = {{-76, 21}, {76, -21}}, textString = "qd"), Text(origin = {27, -57}, lineColor = {0, 0, 255}, extent = {{-73, 19}, {73, -19}}, textString = "Sys")}));
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

        model FControl_TeIqr
          parameter DWT.Units.PerUnit Ls = 3.1 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lm = 3.0 annotation(
            Dialog(group = "Eletrical Data"));
          Modelica.Blocks.Interfaces.RealInput Te annotation(
            Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput Iqr annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Vqs annotation(
            Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-1.77636e-15, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        equation
          Te = Vqs*(-Lm/Ls*Iqr);
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "Iqr(Te)")}));
        end FControl_TeIqr;

        model FControl_QsIdr
          parameter DWT.Units.PerUnit Ls = 3.1 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lm = 3.0 annotation(
            Dialog(group = "Eletrical Data"));
          Modelica.Blocks.Interfaces.RealInput Qs annotation(
            Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealOutput Idr annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Vqs annotation(
            Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-1.77636e-15, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        equation
          Qs = Vqs*((Vqs - Lm*Idr)/Ls);
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "Idr(Qs)")}));
        end FControl_QsIdr;

        model FControl_VccIqg
          // Parameters:
          parameter DWT.Units.PerUnit Ceq = 35.897 annotation(
            Dialog(group = "Eletrical Data"));
          // Modelica pins:
          Modelica.Blocks.Interfaces.RealOutput Iqg annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Vcc annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Pcc annotation(
            Placement(visible = true, transformation(origin = {-50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealInput Vqs annotation(
            Placement(visible = true, transformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        equation
          Vcc = Iqg + 0*(Pcc - Vqs*Iqg);
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {3, 3}, lineColor = {0, 0, 255}, extent = {{-49, 49}, {49, -49}}, textString = "Udg(mdg)")}));
        end FControl_VccIqg;

        model FControl_UqrVqr
          // Parameters:
          parameter DWT.Units.PerUnit Ls = 3.1 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lr = 3.08 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lm = 3.0 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lac = Lr - Lm^2/Ls annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Kc = 0.5/sqrt(2)*1400/(690/sqrt(3)) annotation(
            Dialog(group = "Eletrical Data"));
          // Variables:
          DWT.Units.PerUnit disVqr;
          // Modelica pins:
          Modelica.Blocks.Interfaces.RealInput Wmed annotation(
            Placement(visible = true, transformation(origin = {-90, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {30, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealOutput Vqr annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Vqs annotation(
            Placement(visible = true, transformation(origin = {30, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-90, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealInput Uqr annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Idr annotation(
            Placement(visible = true, transformation(origin = {-30, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
            Placement(visible = true, transformation(origin = {90, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {90, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        equation
          disVqr = (1 - Wmed)*(Lac*Idr + Lm/Ls*Vqs);
          Vqr*Kc*Vccmed = Uqr + disVqr;
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "Uqr(mqr)")}));
        end FControl_UqrVqr;

        model FControl_UdrVdr
          // Parameters:
          parameter DWT.Units.PerUnit Ls = 3.1 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lr = 3.08 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lm = 3.0 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Lac = Lr - Lm^2/Ls annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Kc = 0.5/sqrt(2)*1400/(690/sqrt(3)) annotation(
            Dialog(group = "Eletrical Data"));
          // Variables:
          DWT.Units.PerUnit disVdr;
          // Modelica pins:
          Modelica.Blocks.Interfaces.RealInput Wmed annotation(
            Placement(visible = true, transformation(origin = {-50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealOutput Vdr annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Udr annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Iqr annotation(
            Placement(visible = true, transformation(origin = {0, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
            Placement(visible = true, transformation(origin = {30, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        equation
          disVdr = -(1 - Wmed)*Lac*Iqr;
          Vdr*Kc*Vccmed = Udr + disVdr;
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "Udr(mdr)")}));
        end FControl_UdrVdr;

        model FControl_UqgVqg
          // Parameters:
          parameter DWT.Units.PerUnit Lc = 0.04 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Kc = 0.5/sqrt(2)*1400/(690/sqrt(3)) annotation(
            Dialog(group = "Eletrical Data"));
          // Variables:
          DWT.Units.PerUnit disVqg;
          // Modelica pins:
          Modelica.Blocks.Interfaces.RealOutput Vqg annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Vqs annotation(
            Placement(visible = true, transformation(origin = {30, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealInput Uqg annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Idg annotation(
            Placement(visible = true, transformation(origin = {-30, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
            Placement(visible = true, transformation(origin = {70, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        equation
          disVqg = (-Lc*Idg) - Vqs;
          Vqg*Kc*Vccmed = Uqg + disVqg;
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "Uqg(mqg)")}));
        end FControl_UqgVqg;

        model FControl_UdgVdg
          // Parameters:
          parameter DWT.Units.PerUnit Lc = 0.04 annotation(
            Dialog(group = "Eletrical Data"));
          parameter DWT.Units.PerUnit Kc = 0.5/sqrt(2)*1400/(690/sqrt(3)) annotation(
            Dialog(group = "Eletrical Data"));
          // Variables:
          DWT.Units.PerUnit disVdg;
          // Modelica pins:
          Modelica.Blocks.Interfaces.RealOutput Vdg annotation(
            Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Udg annotation(
            Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Interfaces.RealInput Iqg annotation(
            Placement(visible = true, transformation(origin = {-50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
          Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
            Placement(visible = true, transformation(origin = {50, 120}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        equation
          disVdg = -Lc*Iqg;
          Vdg*Kc*Vccmed = Udg + disVdg;
          annotation(
            Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "Udg(mdg)")}));
        end FControl_UdgVdg;

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
          Modelica.Blocks.Nonlinear.Limiter Limiter1(homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy, uMax = maxMod) annotation(
            Placement(visible = true, transformation(origin = {0, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
          Modelica.Blocks.Nonlinear.Limiter Limiter(homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy, uMax = maxMod) annotation(
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
      end Funcoes;

      model PLL
        parameter Real kp = 10, ki = 10;
        DWT.WindTurbine.ControlModel.Funcoes.T park annotation(
          Placement(visible = true, transformation(origin = {-40, 0}, extent = {{14, -14}, {-14, 14}}, rotation = 0)));
        Modelica.Blocks.Continuous.PI pi(T = kp/ki, initType = Modelica.Blocks.Types.Init.SteadyState, k = kp) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.SteadyState, y(fixed = false)) annotation(
          Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Delta annotation(
          Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealOutput Vqds[2] annotation(
          Placement(visible = true, transformation(origin = {70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Vt[2] annotation(
          Placement(visible = true, transformation(origin = {-88, 8}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      equation
        connect(Vt, park.u) annotation(
          Line(points = {{-88, 8}, {-56, 8}}, color = {0, 0, 127}, thickness = 0.5));
        connect(park.y[2], pi.u) annotation(
          Line(points = {{-24, 0}, {-12, 0}}, color = {0, 0, 127}));
        connect(park.y, Vqds) annotation(
          Line(points = {{-24, 0}, {-20, 0}, {-20, 30}, {70, 30}}, color = {0, 0, 127}, thickness = 0.5));
        connect(pi.y, integrator.u) annotation(
          Line(points = {{12, 0}, {24, 0}}, color = {0, 0, 127}));
        connect(integrator.y, Delta) annotation(
          Line(points = {{48, 0}, {70, 0}}, color = {0, 0, 127}));
        connect(integrator.y, park.theta) annotation(
          Line(points = {{48, 0}, {52, 0}, {52, -28}, {-62, -28}, {-62, -8}, {-56, -8}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(lineColor = {0, 0, 255}, extent = {{-100, 40}, {100, -40}}, textString = "PLL")}),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
      end PLL;

      model ControlMachine
        import SI = Modelica.Units.SI;
        // CTRL Wrm data:
        parameter Units.PerUnit kiWrm = 6.04672 "ki by speed" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpWrm = 15.1168 "kp by speed" annotation(
          Dialog(group = "Control setings"));
        // CTRL Qs data:
        parameter Units.PerUnit kiQs = 1 "ki by reactive power" annotation(
          Dialog(group = "Control setings"));
        // CTRL PLL data:
        parameter Units.PerUnit kiPLL = 200 "ki by PLL" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpPLL = 100 "kp by PLL" annotation(
          Dialog(group = "Control setings"));
        // CTRL RSC data:
        parameter Units.PerUnit kiIqr = 113135.483871 "ki by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIqr = 282.82871 "kp by Iqr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kiIdr = 113135.483871 "ki by Idr" annotation(
          Dialog(group = "Control setings"));
        parameter Units.PerUnit kpIdr = 282.82871 "kp by Idr" annotation(
          Dialog(group = "Control setings"));
        // CTRL GSC data:
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
        // Elec data:
        parameter Units.PerUnit Ls = 3.1 "Stator leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lr = 3.08 "Rotor leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lm = 3.0 "Stator magnetizing inductance" annotation(
          Dialog(group = "Electrical Data"));
        // Parametros conversor:
        parameter DWT.Units.PerUnit Ceq = 35.897 "Capacitor of conversor" annotation(
          Dialog(group = "Eletrical Data"));
        parameter DWT.Units.PerUnit Kc = 0.5/sqrt(2)*1400/(690/sqrt(3)) annotation(
          Dialog(group = "Eletrical Data"));
        //
        //
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mqdr annotation(
          Placement(visible = true, transformation(origin = {104, 50}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mqdg annotation(
          Placement(visible = true, transformation(origin = {102, -54}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealInput Iqdr[2] annotation(
          Placement(visible = true, transformation(origin = {-32, 136}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Iqdg[2] annotation(
          Placement(visible = true, transformation(origin = {18, 136}, extent = {{8, 8}, {-8, -8}}, rotation = 180), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
          Placement(visible = true, transformation(origin = {-118, -56}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput Vqds[2] annotation(
          Placement(visible = true, transformation(origin = {-82, 124}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Qmed annotation(
          Placement(visible = true, transformation(origin = {-112, 16}, extent = {{8, 8}, {-8, -8}}, rotation = 180), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Wmed annotation(
          Placement(visible = true, transformation(origin = {-112, 56}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Wref annotation(
          Placement(visible = true, transformation(origin = {-112, 70}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.PLL pll(ki = kiPLL, kp = kpPLL) annotation(
          Placement(visible = true, transformation(origin = {-60, 124}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regW(ki = kiWrm, kp = kpWrm) annotation(
          Placement(visible = true, transformation(origin = {-90, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regIqr(ki = kiIqr, kp = kpIqr) annotation(
          Placement(visible = true, transformation(origin = {-16, 70}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        Modelica.ComplexBlocks.ComplexMath.RealToComplex real2Complex1 annotation(
          Placement(visible = true, transformation(origin = {84, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.iT ipark annotation(
          Placement(visible = true, transformation(origin = {58, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.T park annotation(
          Placement(visible = true, transformation(origin = {-10, 130}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regIdr(ki = kiIdr, kp = kpIdr) annotation(
          Placement(visible = true, transformation(origin = {-16, 30}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regQ(ki = kiQs, kp = 0) annotation(
          Placement(visible = true, transformation(origin = {-90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.FControl_TeIqr fControl_TeIqr(Lm = Lm, Ls = Ls) annotation(
          Placement(visible = true, transformation(origin = {-64, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.FControl_QsIdr fControl_QsIdr(Lm = Lm, Ls = Ls) annotation(
          Placement(visible = true, transformation(origin = {-64, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.FControl_UqrVqr fControl_UqrVqr(Kc = Kc, Lm = Lm, Lr = Lr, Ls = Ls) annotation(
          Placement(visible = true, transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.FControl_UdrVdr fControl_UdrVdr(Kc = Kc, Lm = Lm, Lr = Lr, Ls = Ls) annotation(
          Placement(visible = true, transformation(origin = {10, 30}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.T park1 annotation(
          Placement(visible = true, transformation(origin = {40, 130}, extent = {{-10, 10}, {10, -10}}, rotation = 180)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regIdg(ki = kiIdg, kp = kpIdg) annotation(
          Placement(visible = true, transformation(origin = {-18, -80}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        Modelica.ComplexBlocks.ComplexMath.RealToComplex real2Complex2 annotation(
          Placement(visible = true, transformation(origin = {82, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM regIqg(ki = kiIqg, kp = kpIqg) annotation(
          Placement(visible = true, transformation(origin = {-18, -30}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.PI_ASTROM pi_astrom3(ki = kiVcc, kp = kpVcc) annotation(
          Placement(visible = true, transformation(origin = {-64, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.iT ipark1 annotation(
          Placement(visible = true, transformation(origin = {56, -54}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant refIdg(k = Idgref) annotation(
          Placement(visible = true, transformation(origin = {-64, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant fVccref(k = Vccref) annotation(
          Placement(visible = true, transformation(origin = {-120, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product quad2 annotation(
          Placement(visible = true, transformation(origin = {-90, -56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.Limiter Limiter1(maxMod = 1) annotation(
          Placement(visible = true, transformation(origin = {-40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.Limiter Limiter2 annotation(
          Placement(visible = true, transformation(origin = {34, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.Limiter Limiter3(maxMod = 1) annotation(
          Placement(visible = true, transformation(origin = {-40, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.FControl_UqgVqg fControl_UqgVqg(Kc = Kc) annotation(
          Placement(visible = true, transformation(origin = {8, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.FControl_UdgVdg fControl_UdgVdg(Kc = Kc) annotation(
          Placement(visible = true, transformation(origin = {8, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.WindTurbine.ControlModel.Funcoes.Limiter Limiter4 annotation(
          Placement(visible = true, transformation(origin = {32, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product quad1 annotation(
          Placement(visible = true, transformation(origin = {-90, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Constant fQgref(k = Qgref) annotation(
          Placement(visible = true, transformation(origin = {-128, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
// Connect Vqs:
        pll.Vqds[1] = fControl_TeIqr.Vqs;
        pll.Vqds[1] = fControl_QsIdr.Vqs;
        pll.Vqds[1] = fControl_UqrVqr.Vqs;
        pll.Vqds[1] = fControl_UqgVqg.Vqs;
// Connect Theta:
        pll.Delta = ipark.theta;
        pll.Delta = ipark1.theta;
// Conect Vccmed:
        Vccmed = fControl_UqrVqr.Vccmed;
        Vccmed = fControl_UdrVdr.Vccmed;
        Vccmed = fControl_UqgVqg.Vccmed;
        Vccmed = fControl_UdgVdg.Vccmed;
// Connect Wrmed
        Wmed = fControl_UqrVqr.Wmed;
        Wmed = fControl_UdrVdr.Wmed;
// Connect Iqr:
        park.y[1] = fControl_UdrVdr.Iqr;
        park.y[1] = regIqr.m;
// Connect Idr:
        park.y[2] = fControl_UqrVqr.Idr;
        park.y[2] = regIdr.m;
// Connect Iqg:
        park1.y[1] = fControl_UdgVdg.Iqg;
        park1.y[1] = regIqg.m;
// Connect Idg:
        park1.y[2] = fControl_UqgVqg.Idg;
        park1.y[2] = regIdg.m;
        connect(Wref, regW.r) annotation(
          Line(points = {{-112, 70}, {-102, 70}}, color = {0, 0, 127}));
        connect(Wmed, regW.m) annotation(
          Line(points = {{-112, 56}, {-90, 56}, {-90, 58}}, color = {0, 0, 127}));
        connect(regW.y, fControl_TeIqr.Te) annotation(
          Line(points = {{-78.8, 70}, {-75, 70}}, color = {0, 0, 127}));
        connect(regQ.y, fControl_QsIdr.Qs) annotation(
          Line(points = {{-78.8, 30}, {-75, 30}}, color = {0, 0, 127}));
        connect(Qmed, regQ.m) annotation(
          Line(points = {{-112, 16}, {-90, 16}, {-90, 19}}, color = {0, 0, 127}));
        connect(Vqds, pll.Vt) annotation(
          Line(points = {{-82, 124}, {-71, 124}}, color = {0, 0, 127}, thickness = 0.5));
        connect(Iqdr, park.u) annotation(
          Line(points = {{-32, 136}, {-21, 136}}, color = {0, 0, 127}, thickness = 0.5));
        connect(regIqr.y, fControl_UqrVqr.Uqr) annotation(
          Line(points = {{-4.8, 70}, {-1, 70}}, color = {0, 0, 127}));
        connect(regIdr.y, fControl_UdrVdr.Udr) annotation(
          Line(points = {{-4.8, 30}, {-1, 30}}, color = {0, 0, 127}));
        connect(real2Complex1.y, Mqdr) annotation(
          Line(points = {{95, 50}, {104, 50}}, color = {85, 170, 255}));
        connect(ipark.y[1], real2Complex1.re) annotation(
          Line(points = {{69, 50}, {69, 56}, {72, 56}}, color = {0, 0, 127}));
        connect(ipark.y[2], real2Complex1.im) annotation(
          Line(points = {{69, 50}, {69, 44}, {72, 44}}, color = {0, 0, 127}));
        connect(Iqdg, park1.u) annotation(
          Line(points = {{18, 136}, {29, 136}}, color = {0, 0, 127}, thickness = 0.5));
        connect(pll.Delta, park.theta) annotation(
          Line(points = {{-49, 118}, {-30, 118}, {-30, 124}, {-21, 124}}, color = {0, 0, 127}));
        connect(ipark1.y[1], real2Complex2.re) annotation(
          Line(points = {{67, -54}, {67, -48}, {70, -48}}, color = {0, 0, 127}));
        connect(ipark1.y[2], real2Complex2.im) annotation(
          Line(points = {{67, -54}, {67, -60}, {70, -60}}, color = {0, 0, 127}));
        connect(real2Complex2.y, Mqdg) annotation(
          Line(points = {{93, -54}, {102, -54}}, color = {85, 170, 255}));
        connect(Vccmed, quad2.u1) annotation(
          Line(points = {{-118, -56}, {-110, -56}, {-110, -50}, {-102, -50}}, color = {0, 0, 127}));
        connect(Vccmed, quad2.u2) annotation(
          Line(points = {{-118, -56}, {-110, -56}, {-110, -62}, {-102, -62}}, color = {0, 0, 127}));
        connect(fControl_TeIqr.Iqr, Limiter1.u[1]) annotation(
          Line(points = {{-53, 70}, {-51, 70}, {-51, 50}}, color = {0, 0, 127}));
        connect(fControl_QsIdr.Idr, Limiter1.u[2]) annotation(
          Line(points = {{-53, 30}, {-51, 30}, {-51, 50}}, color = {0, 0, 127}));
        connect(Limiter1.y[2], regIdr.r) annotation(
          Line(points = {{-29.4, 50}, {-27, 50}, {-27, 30}}, color = {0, 0, 127}));
        connect(Limiter1.y[1], regIqr.r) annotation(
          Line(points = {{-29.4, 50}, {-27, 50}, {-27, 70}}, color = {0, 0, 127}));
        connect(fControl_UqrVqr.Vqr, Limiter2.u[1]) annotation(
          Line(points = {{21, 70}, {23, 70}, {23, 50}}, color = {0, 0, 127}));
        connect(fControl_UdrVdr.Vdr, Limiter2.u[2]) annotation(
          Line(points = {{21, 30}, {23, 30}, {23, 50}}, color = {0, 0, 127}));
        connect(Limiter2.y, ipark.u) annotation(
          Line(points = {{44.6, 50}, {47, 50}}, color = {0, 0, 127}, thickness = 0.5));
        connect(refIdg.y, Limiter3.u[2]) annotation(
          Line(points = {{-53, -80}, {-51, -80}, {-51, -54}}, color = {0, 0, 127}));
        connect(Limiter3.y[1], regIqg.r) annotation(
          Line(points = {{-29, -54}, {-29, -30}}, color = {0, 0, 127}));
        connect(Limiter3.y[2], regIdg.r) annotation(
          Line(points = {{-29, -54}, {-29, -80}}, color = {0, 0, 127}));
        connect(regIqg.y, fControl_UqgVqg.Uqg) annotation(
          Line(points = {{-6.8, -30}, {-3, -30}}, color = {0, 0, 127}));
        connect(regIdg.y, fControl_UdgVdg.Udg) annotation(
          Line(points = {{-6.8, -80}, {-3, -80}}, color = {0, 0, 127}));
        connect(Limiter4.y, ipark1.u) annotation(
          Line(points = {{42.6, -54}, {45, -54}}, color = {0, 0, 127}, thickness = 0.5));
        connect(fControl_UqgVqg.Vqg, Limiter4.u[1]) annotation(
          Line(points = {{19, -30}, {21, -30}, {21, -54}}, color = {0, 0, 127}));
        connect(fControl_UdgVdg.Vdg, Limiter4.u[2]) annotation(
          Line(points = {{19, -80}, {21, -80}, {21, -54}}, color = {0, 0, 127}));
        connect(quad2.y, pi_astrom3.m) annotation(
          Line(points = {{-79, -56}, {-64, -56}, {-64, -41}}, color = {0, 0, 127}));
        connect(pi_astrom3.y, Limiter3.u[1]) annotation(
          Line(points = {{-52.8, -30}, {-51, -30}, {-51, -54}}, color = {0, 0, 127}));
        connect(fVccref.y, quad1.u1) annotation(
          Line(points = {{-109, -30}, {-108, -30}, {-108, -24}, {-102, -24}}, color = {0, 0, 127}));
        connect(fVccref.y, quad1.u2) annotation(
          Line(points = {{-109, -30}, {-108, -30}, {-108, -36}, {-102, -36}}, color = {0, 0, 127}));
        connect(quad1.y, pi_astrom3.r) annotation(
          Line(points = {{-78, -30}, {-75, -30}}, color = {0, 0, 127}));
        connect(pll.Delta, park1.theta) annotation(
          Line(points = {{-48, 118}, {20, 118}, {20, 124}, {30, 124}}, color = {0, 0, 127}));
        connect(fQgref.y, regQ.r) annotation(
          Line(points = {{-116, 30}, {-102, 30}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(origin = {0, 40}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 40}, {100, -40}}), Text(origin = {0, 38}, lineColor = {0, 0, 255}, extent = {{-60, 20}, {60, -20}}, textString = "ELEC CTRL")}),
          Diagram(graphics = {Text(origin = {-41, 131}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-10, 2}, {10, -2}}, textString = "Vqsmed", textStyle = {TextStyle.Bold, TextStyle.Bold}), Text(origin = {9, 133}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-10, 2}, {10, -2}}, textString = "Iqrmed", textStyle = {TextStyle.Bold}), Text(origin = {9, 127}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-10, 2}, {10, -2}}, textString = "Idrmed", textStyle = {TextStyle.Bold}), Text(origin = {59, 133}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-10, 2}, {10, -2}}, textString = "Iqgmed", textStyle = {TextStyle.Bold}), Text(origin = {59, 127}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-10, 2}, {10, -2}}, textString = "Idgmed", textStyle = {TextStyle.Bold})}));
      end ControlMachine;

      model ControlTurbine
        import pi = Modelica.Constants.pi;
        Modelica.Blocks.Interfaces.RealInput Vw annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-112, 40}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput beta annotation(
          Placement(visible = true, transformation(origin = {-10, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealOutput Wrm_opt annotation(
          Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
          Icon(graphics = {Rectangle(origin = {0, 40}, lineColor = {0, 0, 255}, fillColor = {46, 52, 54}, extent = {{-100, 40}, {100, -40}}), Text(origin = {1, 40}, lineColor = {0, 0, 255}, extent = {{-63, 36}, {63, -36}}, textString = "MECH CTRL")}),
          experiment(StartTime = 0, StopTime = 510, Tolerance = 1e-06, Interval = 0.005));
      end ControlTurbine;
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
        S = pin_p.v*conj(pin_p.i);
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
        Modelica.Blocks.Sources.Ramp ramp(duration = 60, height = 4, offset = 7, startTime = 5) annotation(
          Placement(visible = true, transformation(origin = {-32, 16}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
        DFIG dfig(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter DWT.WindTurbine.Interfaces.DWTData smData(MVAs = data.Sbase, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/betaR4.mat") annotation(
          Placement(visible = true, transformation(origin = {-10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(ramp.y, dfig.Vw) annotation(
          Line(points = {{-25, 16}, {-10, 16}}, color = {0, 0, 127}));
        connect(dfig.pin_DFIG, voltageSource.p) annotation(
          Line(points = {{12, 16}, {31, 16}}, color = {0, 0, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.001));
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
        parameter DWT.WindTurbine.Interfaces.DWTData smData(MVAs = data.Sbase, Qgref = -0.5, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/betaR4.mat") annotation(
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
        parameter DWT.WindTurbine.Interfaces.DWTData smData(MVAs = data.Sbase, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/betaR4.mat") annotation(
          Placement(visible = true, transformation(origin = {-10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        DWT.Circuit.Sources.ControlledVoltageSource controlledVoltageSource annotation(
          Placement(visible = true, transformation(origin = {50, 6}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        if time < 1 then
          controlledVoltageSource.v = Complex(1, 0);
        elseif time > 1 and time <= 2 then
          controlledVoltageSource.v = Complex(0.9, 0);
        else
          controlledVoltageSource.v = Complex(1, 0);
        end if;
        connect(ramp.y, dfig.Vw) annotation(
          Line(points = {{-23, 16}, {-10, 16}}, color = {0, 0, 127}));
        connect(dfig.pin_DFIG, controlledVoltageSource.p) annotation(
          Line(points = {{12, 16}, {50, 16}}, color = {0, 0, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0005));
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
        parameter DWT.WindTurbine.Interfaces.DWTData smData(MVAs = data.Sbase, Qgref = -0.5, Wb = data.wb, fileNameR2 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/omegaR2.mat", fileNameR4 = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/LookupTables/betaR4.mat") annotation(
          Placement(visible = true, transformation(origin = {-10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Circuit.Basic.SeriesImpedance seriesImpedance(r = 0, x = 0) annotation(
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