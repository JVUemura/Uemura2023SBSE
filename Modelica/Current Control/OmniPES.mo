package OmniPES
  package Units
    type PerUnit = Real(unit = "pu");
    operator record CPerUnit = Complex(redeclare PerUnit re, redeclare PerUnit im);
    type ActivePower = Real(final quantity = "Power", final unit = "MW");
    type ApparentPower = Real(final quantity = "Power", final unit = "MVA");
    type ReactivePower = Real(final quantity = "Power", final unit = "Mvar");
    type Voltage = Real(final quantity = "Voltage", final unit = "kV");
    type Current = Real(final quantity = "Current", final unit = "kA");
  end Units;

  model SystemData
    import Modelica.Units.SI;
    import Modelica.Constants.pi;
    parameter OmniPES.Units.ApparentPower Sbase = 100 annotation(
      Dialog(group = "Base Quantities"));
    parameter SI.Frequency fb = 60 annotation(
      Dialog(group = "Base Quantities"));
    parameter SI.AngularVelocity wb = 2 * pi * fb;
    annotation(
      defaultComponentName = "data",
      defaultComponentPrefixes = "inner",
      missingInnerMessage = "The System object is missing, please drag it on the top layer of your model",
      Icon(coordinateSystem(initialScale = 0.1), graphics = {Rectangle(fillColor = {255, 255, 0}, fillPattern = FillPattern.Solid, extent = {{-80, 70}, {80, -70}}), Text(extent = {{-60, 40}, {60, -40}}, textString = "System")}),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
  end SystemData;

  package Math
    extends Modelica.Icons.FunctionsPackage;

    function sys2qd
      extends Modelica.Icons.Function;
      input Complex A;
      input Modelica.Units.SI.Angle delta;
      output Complex Aqd;
      import Modelica.ComplexMath.j;
    algorithm
      Aqd := A * Modelica.ComplexMath.exp(-j * delta);
    end sys2qd;

    function polar2cart
      extends Modelica.Icons.Function;
      import Modelica.ComplexMath.exp;
      import Modelica.ComplexMath.j;
      import Modelica.Constants.pi;
      input Real mag "Absolute value of the complex";
      input Modelica.Units.SI.Angle phase "Phase angle of the complex";
      output Complex z "Resultant complex number";
    algorithm
      z := mag * exp(j * phase * pi / 180);
    end polar2cart;
  end Math;

  package Circuit
    package Interfaces
      extends Modelica.Icons.InterfacesPackage;

      connector PositivePin
        import OmniPES.Units;
        Units.CPerUnit v "Positive node voltage";
        flow Units.CPerUnit i "Sum of currents flowing into node";
        annotation(
          defaultComponentName = "pin_p",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-160, 110}, {40, 50}}, lineColor = {0, 0, 255}, textString = "%name")}));
      end PositivePin;

      connector NegativePin
        import OmniPES.Units;
        Units.CPerUnit v "Negative node voltage";
        flow Units.CPerUnit i "Sum of currents flowing into node";
        annotation(
          defaultComponentName = "pin_n",
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid)}),
          Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-40, 40}, {40, -40}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-40, 110}, {160, 50}}, textString = "%name", lineColor = {0, 0, 255})}));
      end NegativePin;

      partial model SeriesComponent
        import OmniPES.Units;
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

      model Bus
        import OmniPES.Units;
        import Modelica.Units.SI;
        OmniPES.Circuit.Interfaces.PositivePin p(v.re(start = 1.0)) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-100, -100}, {100, 100}}, rotation = 0), iconTransformation(origin = {-2, -20}, extent = {{-15, -150}, {15, 150}}, rotation = 0)));
        Units.PerUnit V(start = 1.0);
        SI.Angle angle(start = 0);
      equation
        V ^ 2 = p.v.re ^ 2 + p.v.im ^ 2;
        p.v.im = p.v.re * tan(angle);
        p.i = Complex(0);
        annotation(
          Icon(graphics = {Rectangle(origin = {-7, 3}, extent = {{1, 97}, {13, -105}}), Text(origin = {-5, 163}, lineColor = {0, 0, 255}, extent = {{-83, 41}, {83, -41}}, textString = "%name")}));
      end Bus;

      partial model ShuntComponent
        import OmniPES.Units;
        OmniPES.Circuit.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Units.CPerUnit v "Node voltage across this shunt element.";
        Units.CPerUnit i "Current flowing towards the reference node.";
      equation
        v = p.v;
        i = p.i;
      end ShuntComponent;

      model IdealTransformer
        parameter Real a = 1.0 "Transformer ratio";
        PositivePin p annotation(
          Placement(visible = true, transformation(origin = {-104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        NegativePin n annotation(
          Placement(visible = true, transformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        p.v = a * n.v;
        n.i + a * p.i = Complex(0);
        annotation(
          Icon(graphics = {Line(origin = {72, 0}, points = {{-27, 0}, {30, 0}}), Line(origin = {-72, 0}, points = {{-30, 0}, {27, 0}}), Ellipse(origin = {-17, 0}, extent = {{-28, 28}, {28, -28}}, endAngle = 360), Ellipse(origin = {17, 0}, extent = {{-28, 28}, {28, -28}}, endAngle = 360), Ellipse(origin = {-48, 28}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}, endAngle = 360)}));
      end IdealTransformer;
    end Interfaces;

    package Sources
      extends Modelica.Icons.SourcesPackage;

      model VoltageSource
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        import OmniPES.Math.polar2cart;
        //
        parameter OmniPES.Units.PerUnit magnitude = 1.0;
        parameter Modelica.Units.NonSI.Angle_deg angle = 0.0;
        OmniPES.Units.CPerUnit S;
      equation
        v = polar2cart(magnitude, angle);
        S = -v * conj(i);
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}, endAngle = 360), Line(origin = {-73, 0}, points = {{-13, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}})}, coordinateSystem(initialScale = 0.1)));
      end VoltageSource;

      model CurrentSource
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import OmniPES.Math.polar2cart;
        parameter OmniPES.Units.PerUnit magnitude = 0.0;
        parameter Modelica.Units.NonSI.Angle_deg angle = 0.0;
      equation
//  if time < 1 then
//    i = 0;
//  else
        i = if time < 1 then Complex(0.0) else polar2cart(magnitude, angle + 180);
//  end
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}, endAngle = 360), Line(origin = {-73, 0}, points = {{-13, 0}, {13, 0}}), Line(origin = {-18.0759, 0}, points = {{75.235, 0}, {11.235, 0}, {11.235, 20}, {-20.765, 0}, {11.235, -20}, {11.235, 0}}), Line(origin = {93, 0}, points = {{-21, 0}, {21, 0}}), Line(origin = {114, -1}, points = {{0, 35}, {0, -35}}), Line(origin = {130, -1}, points = {{0, 21}, {0, -19}}), Line(origin = {144, -1}, points = {{0, 9}, {0, -5}})}, coordinateSystem(initialScale = 0.1)));
      end CurrentSource;

      model PVSource
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        parameter OmniPES.Units.ActivePower Psp;
        parameter OmniPES.Units.PerUnit Vsp = 1.0;
        outer OmniPES.SystemData data;
        OmniPES.Units.CPerUnit S;
      equation
        S = -v * conj(i);
        S.re = Psp / data.Sbase;
        Vsp ^ 2 = v.re ^ 2 + v.im ^ 2;
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}), Line(origin = {-73, 0}, points = {{-13, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}}), Text(origin = {80.4, -71.35}, rotation = 90, extent = {{-25.35, 29.6}, {26.65, -18.4}}, textString = "PV")}, coordinateSystem(initialScale = 0.1)));
      end PVSource;

      model PQSource
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        parameter OmniPES.Units.ActivePower P;
        parameter OmniPES.Units.ReactivePower Q;
        outer OmniPES.SystemData data;
        OmniPES.Units.CPerUnit S;
      equation
        S = -v * conj(i);
        S.re = P / data.Sbase;
        S.im = Q / data.Sbase;
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}, endAngle = 360), Line(origin = {-73, 0}, points = {{-13, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}}), Text(origin = {91, -75}, rotation = 90, extent = {{-39, 37}, {41, -23}}, textString = "PQ")}, coordinateSystem(initialScale = 0.1)));
      end PQSource;

      model PVSource_Qlim
        outer OmniPES.SystemData data;
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        parameter OmniPES.Units.ActivePower Psp;
        parameter OmniPES.Units.PerUnit Vsp = 1.0;
        parameter OmniPES.Units.ReactivePower Qmin = -1e5;
        parameter OmniPES.Units.ReactivePower Qmax = +1e5;
        parameter Real tolq = 1e-3;
        parameter Real tolv = 1e-3;
        parameter Real inc = 1e5;
        OmniPES.Units.CPerUnit S(re(start = Psp / data.Sbase), im(start = 0));
        OmniPES.Units.PerUnit Vabs(start = 1);
        Real ch1(start = 0), ch2(start = 0), ch3(start = 0), ch4(start = 0);
      protected
        parameter Real lim_max = Qmax / data.Sbase - tolq;
        parameter Real lim_min = Qmin / data.Sbase + tolq;
        parameter Real lim_sup = Vsp + tolv;
        parameter Real lim_inf = Vsp - tolv;
      equation
        S = -v * conj(i);
        S.re = Psp / data.Sbase;
        (1 - ch1 * ch3) * (1 - ch2 * ch4) * (Vabs - Vsp) + ch1 * ch3 * (1 - ch2 * ch4) * (S.im - Qmax / data.Sbase) + (1 - ch1 * ch3) * (ch2 * ch4) * (S.im - Qmin / data.Sbase) = 0;
        Vabs ^ 2 = v.re ^ 2 + v.im ^ 2;
      algorithm
        ch1 := 1 / (1 + exp(-inc * (S.im - lim_max)));
        ch2 := 1 / (1 + exp(inc * (S.im - lim_min)));
        ch3 := 1 / (1 + exp(inc * (Vabs - lim_sup)));
        ch4 := 1 / (1 + exp(-inc * (Vabs - lim_inf)));
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}, endAngle = 360), Line(origin = {-73, 0}, points = {{-25, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}}), Text(origin = {75, 65}, rotation = -90, extent = {{-33, 25}, {33, -25}}, textString = "PV\nQlim")}, coordinateSystem(initialScale = 0.1)));
      end PVSource_Qlim;

      model ControlledVoltageSource
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        import OmniPES.Math.polar2cart;
        //
        OmniPES.Units.CPerUnit S;
        Modelica.ComplexBlocks.Interfaces.ComplexInput u annotation(
          Placement(visible = true, transformation(origin = {-62, -48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {6, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        v = u;
        S = -v * conj(i);
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}, endAngle = 360), Line(origin = {-73, 0}, points = {{-25, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}})}, coordinateSystem(initialScale = 0.1)));
      end ControlledVoltageSource;

      model ControlledPQSource
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        Modelica.ComplexBlocks.Interfaces.ComplexInput u annotation(
          Placement(visible = true, transformation(origin = {-62, -48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {6, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        u = -v * conj(i);
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}, endAngle = 360), Line(origin = {-73, 0}, points = {{-13, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {-4, -22}, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {16, 22}, rotation = 180, points = {{12, -22}, {8, -22}, {2, -20}, {-4, -16}, {-10, -10}, {-12, -2}, {-12, 4}, {-10, 8}, {-6, 14}, {-2, 18}, {2, 20}, {8, 22}, {12, 22}, {12, 22}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}}), Text(origin = {91, -75}, rotation = 90, extent = {{-39, 37}, {41, -23}}, textString = "PQ")}, coordinateSystem(initialScale = 0.1)));
      end ControlledPQSource;

      model ControlledCurrentSource
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        import Modelica.ComplexMath.conj;
        import OmniPES.Math.polar2cart;
        //
        OmniPES.Units.CPerUnit S;
        Modelica.ComplexBlocks.Interfaces.ComplexInput u annotation(
          Placement(visible = true, transformation(origin = {-62, -48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {6, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        i = -u;
        S = -v * conj(i);
        annotation(
          Icon(graphics = {Ellipse(origin = {6, 1}, extent = {{-66, 67}, {66, -67}}), Line(origin = {-73, 0}, points = {{-25, 0}, {13, 0}}), Line(origin = {81, 0}, points = {{-9, 0}, {9, 0}}), Line(origin = {98, 0}, points = {{-10, 0}, {10, 0}}), Line(origin = {108, -1}, points = {{0, 33}, {0, -33}}), Line(origin = {122, -1}, points = {{0, 19}, {0, -19}}), Line(origin = {134, 1}, points = {{0, 3}, {0, -7}}), Line(origin = {-18.0759, 0}, points = {{75.235, 0}, {11.235, 0}, {11.235, 20}, {-20.765, 0}, {11.235, -20}, {11.235, 0}})}, coordinateSystem(initialScale = 0.1)));
      end ControlledCurrentSource;
    end Sources;

    package Basic
      extends Modelica.Icons.Package;

      model SeriesImpedance
        extends OmniPES.Circuit.Interfaces.SeriesComponent;
        parameter OmniPES.Units.PerUnit r = 0.0;
        parameter OmniPES.Units.PerUnit x = 0.0;
      equation
        v = Complex(r, x) * i;
        annotation(
          Icon(graphics = {Rectangle(origin = {1, -1}, extent = {{-61, 35}, {61, -35}}), Line(origin = {-73, 0}, points = {{13, 0}, {-13, 0}}), Line(origin = {76, 0}, points = {{-14, 0}, {14, 0}})}, coordinateSystem(initialScale = 0.1)));
      end SeriesImpedance;

      model SeriesAdmittance
        extends OmniPES.Circuit.Interfaces.SeriesComponent;
        parameter OmniPES.Units.PerUnit g;
        parameter OmniPES.Units.PerUnit b;
      equation
        i = Complex(g, b) * v;
        annotation(
          Icon(graphics = {Rectangle(origin = {1, -1}, fillColor = {211, 215, 207}, fillPattern = FillPattern.Solid, extent = {{-61, 35}, {61, -35}}), Line(origin = {-73, 0}, points = {{13, 0}, {-13, 0}}), Line(origin = {76, 0}, points = {{-14, 0}, {14, 0}})}, coordinateSystem(initialScale = 0.1)));
      end SeriesAdmittance;

      model Ground
        OmniPES.Circuit.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        p.v.re = 0;
        p.v.im = 0;
        annotation(
          Diagram,
          Icon(graphics = {Line(origin = {0, -40}, points = {{-60, 0}, {60, 0}, {60, 0}}), Line(origin = {0, -60}, points = {{-40, 0}, {40, 0}}), Line(origin = {0, -80}, points = {{-20, 0}, {20, 0}}), Line(origin = {0, -15}, points = {{0, 5}, {0, -25}})}, coordinateSystem(initialScale = 0.1)));
      end Ground;

      model Shunt_Capacitor
        parameter OmniPES.Units.ReactivePower NominalPower;
        outer OmniPES.SystemData data;
        extends OmniPES.Circuit.Basic.ShuntAdmittance(g = 0, b = NominalPower / data.Sbase);
        annotation(
          Icon(coordinateSystem(initialScale = 0.1), graphics = {Line(origin = {-21.0002, -2.2362e-05}, points = {{-13, 0}, {13, 0}}), Line(origin = {20.9998, -2.2362e-05}, points = {{-13, 0}, {13, 0}}), Line(origin = {-8.00024, -3.00002}, points = {{0, 23}, {0, -17}}), Line(origin = {7.99976, -3.00002}, points = {{0, 23}, {0, -17}})}));
      end Shunt_Capacitor;

      model ShuntAdmittance
        extends OmniPES.Circuit.Interfaces.ShuntComponent;
        //protected
        parameter OmniPES.Units.PerUnit g;
        parameter OmniPES.Units.PerUnit b;
      equation
        i = Complex(g, b) * v;
        annotation(
          Icon(graphics = {Rectangle(origin = {1, -1}, fillColor = {211, 215, 207}, fillPattern = FillPattern.Solid, extent = {{-61, 35}, {61, -35}}), Line(origin = {-73, 0}, points = {{13, 0}, {-27, 0}}), Line(origin = {76, 0}, points = {{-14, 0}, {24, 0}}), Line(origin = {112, 0}, points = {{0, 22}, {0, -22}}), Line(origin = {100, 0}, points = {{0, 30}, {0, -30}}), Line(origin = {122, 0}, points = {{0, 12}, {0, -12}})}, coordinateSystem(initialScale = 0.1)));
      end ShuntAdmittance;

      model TwoWindingTransformer
        outer OmniPES.SystemData data;
        parameter OmniPES.Units.ApparentPower NominalMVA = data.Sbase;
        parameter OmniPES.Units.PerUnit r = 0;
        parameter OmniPES.Units.PerUnit x;
        parameter Real tap = 1.0;
        OmniPES.Circuit.Interfaces.PositivePin p(v.re(start = 1)) annotation(
          Placement(visible = true, transformation(origin = {-66, 14}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin n(v.re(start = 1)) annotation(
          Placement(visible = true, transformation(origin = {56, 14}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Basic.SeriesImpedance Z(r = r * data.Sbase / NominalMVA, x = x * data.Sbase / NominalMVA) annotation(
          Placement(visible = true, transformation(origin = {26, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.IdealTransformer idealTransformer(a = tap) annotation(
          Placement(visible = true, transformation(origin = {-30, 14}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
      equation
        connect(p, idealTransformer.p) annotation(
          Line(points = {{-66, 14}, {-46, 14}}, color = {0, 0, 255}));
        connect(idealTransformer.n, Z.p) annotation(
          Line(points = {{-14.6, 14}, {15.4, 14}}, color = {0, 0, 255}));
        connect(Z.n, n) annotation(
          Line(points = {{36, 13.8}, {56, 13.8}}, color = {0, 0, 255}));
        annotation(
          Icon(graphics = {Rectangle(origin = {0, -1}, extent = {{-100, 61}, {100, -61}}), Ellipse(origin = {-17, 0}, extent = {{-28, 28}, {28, -28}}, endAngle = 360), Ellipse(origin = {17, 0}, extent = {{-28, 28}, {28, -28}}, endAngle = 360), Line(origin = {-72, 0}, points = {{-30, 0}, {27, 0}}), Line(origin = {72, 0}, points = {{-27, 0}, {30, 0}}), Ellipse(origin = {-60, 40}, fillPattern = FillPattern.Solid, extent = {{-5, 5}, {5, -5}}, endAngle = 360)}, coordinateSystem(initialScale = 0.1)));
      end TwoWindingTransformer;

      model TLine
        outer OmniPES.SystemData data;
        parameter OmniPES.Units.PerUnit r;
        parameter OmniPES.Units.PerUnit x;
        parameter OmniPES.Units.ReactivePower Q;
        OmniPES.Circuit.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {-40, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin n annotation(
          Placement(visible = true, transformation(origin = {40, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Basic.SeriesImpedance Z(r = r, x = x) annotation(
          Placement(visible = true, transformation(origin = {0, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Basic.Shunt_Capacitor Ypp(NominalPower = Q / 2) annotation(
          Placement(visible = true, transformation(origin = {-26, 26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Basic.Shunt_Capacitor Ynn(NominalPower = Q / 2) annotation(
          Placement(visible = true, transformation(origin = {24, 28}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        connect(Z.n, n) annotation(
          Line(points = {{10, 52}, {40, 52}}, color = {0, 0, 255}));
        connect(p, Z.p) annotation(
          Line(points = {{-40, 52}, {-10, 52}}, color = {0, 0, 255}));
        connect(Ypp.p, Z.p) annotation(
          Line(points = {{-26, 36}, {-26, 44}, {-10, 44}, {-10, 52}}, color = {0, 0, 255}));
        connect(Ynn.p, Z.n) annotation(
          Line(points = {{24, 38}, {24, 44}, {10, 44}, {10, 52}}, color = {0, 0, 255}));
        annotation(
          Icon(graphics = {Rectangle(origin = {0, -1}, extent = {{-100, 61}, {100, -61}}), Line(origin = {-80, 30}, points = {{-20, 0}, {20, 0}}), Line(origin = {-60, 25}, points = {{0, 5}, {0, -5}, {0, -5}}), Rectangle(origin = {-60, 0}, extent = {{-10, 20}, {10, -20}}), Line(origin = {-60, -30}, points = {{0, 10}, {0, -10}}), Line(origin = {-60, -40}, points = {{-20, 0}, {20, 0}}), Line(origin = {-60, -46}, points = {{-12, 0}, {12, 0}}), Line(origin = {-60, -50}, points = {{-4, 0}, {4, 0}}), Rectangle(origin = {60, 0}, extent = {{-10, 20}, {10, -20}}), Line(origin = {60, 25}, points = {{0, 5}, {0, -5}, {0, -5}}), Line(origin = {60, -30}, points = {{0, 10}, {0, -10}}), Line(origin = {60, -40}, points = {{-20, 0}, {20, 0}}), Line(origin = {60, -46}, points = {{-12, 0}, {12, 0}}), Line(origin = {60, -50}, points = {{-4, 0}, {4, 0}}), Rectangle(origin = {0, 30}, rotation = -90, extent = {{-10, 20}, {10, -20}}), Line(origin = {-40, 30}, points = {{-20, 0}, {20, 0}}), Line(origin = {40, 30}, points = {{20, 0}, {-20, 0}}), Line(origin = {80, 30}, points = {{-20, 0}, {20, 0}})}, coordinateSystem(initialScale = 0.1)));
      end TLine;

      model TLine_switched
        outer OmniPES.SystemData data;
        parameter OmniPES.Units.PerUnit r;
        parameter OmniPES.Units.PerUnit x;
        parameter OmniPES.Units.ReactivePower Q;
        OmniPES.Circuit.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {-86, 54}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin n annotation(
          Placement(visible = true, transformation(origin = {90, 54}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Basic.SeriesImpedance Z(r = r, x = x) annotation(
          Placement(visible = true, transformation(origin = {0, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Basic.Shunt_Capacitor Ypp(NominalPower = Q / 2) annotation(
          Placement(visible = true, transformation(origin = {-12, 28}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Basic.Shunt_Capacitor Ynn(NominalPower = Q / 2) annotation(
          Placement(visible = true, transformation(origin = {12, 28}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        parameter Real t_open = 0.3;
        Circuit.Switches.TimedBreaker brk_p(t_open = t_open) annotation(
          Placement(visible = true, transformation(origin = {-46, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Switches.TimedBreaker brk_n(t_open = t_open) annotation(
          Placement(visible = true, transformation(origin = {48, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(Z.p, Ypp.p) annotation(
          Line(points = {{-10, 52}, {-12, 52}, {-12, 38}}, color = {0, 0, 255}));
        connect(Z.n, Ynn.p) annotation(
          Line(points = {{10, 52}, {12, 52}, {12, 38}}, color = {0, 0, 255}));
        connect(p, brk_p.p) annotation(
          Line(points = {{-86, 54}, {-56, 54}, {-56, 52}}, color = {0, 0, 255}));
        connect(brk_p.n, Z.p) annotation(
          Line(points = {{-36, 52}, {-10, 52}}, color = {0, 0, 255}));
        connect(Z.n, brk_n.p) annotation(
          Line(points = {{10, 52}, {38, 52}}, color = {0, 0, 255}));
        connect(brk_n.n, n) annotation(
          Line(points = {{58, 52}, {74, 52}, {74, 54}, {90, 54}}, color = {0, 0, 255}));
        annotation(
          Icon(graphics = {Rectangle(origin = {0, -1}, extent = {{-100, 61}, {100, -61}}), Line(origin = {-80, 30}, points = {{-20, 0}, {20, 0}}), Line(origin = {-60, 25}, points = {{0, 5}, {0, -5}, {0, -5}}), Rectangle(origin = {-60, 0}, extent = {{-10, 20}, {10, -20}}), Line(origin = {-60, -30}, points = {{0, 10}, {0, -10}}), Line(origin = {-60, -40}, points = {{-20, 0}, {20, 0}}), Line(origin = {-60, -46}, points = {{-12, 0}, {12, 0}}), Line(origin = {-60, -50}, points = {{-4, 0}, {4, 0}}), Rectangle(origin = {60, 0}, extent = {{-10, 20}, {10, -20}}), Line(origin = {60, 25}, points = {{0, 5}, {0, -5}, {0, -5}}), Line(origin = {60, -30}, points = {{0, 10}, {0, -10}}), Line(origin = {60, -40}, points = {{-20, 0}, {20, 0}}), Line(origin = {60, -46}, points = {{-12, 0}, {12, 0}}), Line(origin = {60, -50}, points = {{-4, 0}, {4, 0}}), Rectangle(origin = {0, 30}, rotation = -90, extent = {{-10, 20}, {10, -20}}), Line(origin = {-40, 30}, points = {{-20, 0}, {20, 0}}), Line(origin = {40, 30}, points = {{20, 0}, {-20, 0}}), Line(origin = {80, 30}, points = {{-20, 0}, {20, 0}}), Text(origin = {-2, -18}, extent = {{38, -38}, {-38, 38}}, textString = "SW")}, coordinateSystem(initialScale = 0.1)));
      end TLine_switched;

      model TLine_eq
        outer OmniPES.SystemData data;
        parameter OmniPES.Units.PerUnit r;
        parameter OmniPES.Units.PerUnit x;
        parameter OmniPES.Units.ReactivePower Q;
        OmniPES.Circuit.Interfaces.PositivePin p annotation(
          Placement(visible = true, transformation(origin = {-40, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin n annotation(
          Placement(visible = true, transformation(origin = {40, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        //protected
        parameter Complex ysh(re = 0, im = Q / 2);
        parameter Complex ykm = 1 / Complex(r, x);
      equation
//  p.i = ykm*(p.v - n.v) + ysh*p.v;
//  n.i = ykm*(n.v - p.v) + ysh*n.v;
        p.i.re = ykm.re * (p.v.re - n.v.re) - ykm.im * (p.v.im - n.v.im) - ysh.im * p.v.im;
        p.i.im = ykm.re * (p.v.im - n.v.im) + ykm.im * (p.v.re - n.v.re) + ysh.im * p.v.re;
        n.i.re = ykm.re * (n.v.re - p.v.re) - ykm.im * (n.v.im - p.v.im) - ysh.im * n.v.im;
        n.i.im = ykm.re * (n.v.im - p.v.im) + ykm.im * (n.v.re - p.v.re) + ysh.im * p.v.re;
        annotation(
          Icon(graphics = {Rectangle(origin = {0, -1}, extent = {{-100, 61}, {100, -61}}), Line(origin = {-80, 30}, points = {{-20, 0}, {20, 0}}), Line(origin = {-60, 25}, points = {{0, 5}, {0, -5}, {0, -5}}), Rectangle(origin = {-60, 0}, extent = {{-10, 20}, {10, -20}}), Line(origin = {-60, -30}, points = {{0, 10}, {0, -10}}), Line(origin = {-60, -40}, points = {{-20, 0}, {20, 0}}), Line(origin = {-60, -46}, points = {{-12, 0}, {12, 0}}), Line(origin = {-60, -50}, points = {{-4, 0}, {4, 0}}), Rectangle(origin = {60, 0}, extent = {{-10, 20}, {10, -20}}), Line(origin = {60, 25}, points = {{0, 5}, {0, -5}, {0, -5}}), Line(origin = {60, -30}, points = {{0, 10}, {0, -10}}), Line(origin = {60, -40}, points = {{-20, 0}, {20, 0}}), Line(origin = {60, -46}, points = {{-12, 0}, {12, 0}}), Line(origin = {60, -50}, points = {{-4, 0}, {4, 0}}), Rectangle(origin = {0, 30}, rotation = -90, extent = {{-10, 20}, {10, -20}}), Line(origin = {-40, 30}, points = {{-20, 0}, {20, 0}}), Line(origin = {40, 30}, points = {{20, 0}, {-20, 0}}), Line(origin = {80, 30}, points = {{-20, 0}, {20, 0}})}, coordinateSystem(initialScale = 0.1)));
      end TLine_eq;
    end Basic;

    package Switches
      model Breaker
        extends OmniPES.Circuit.Switches.Interfaces.BasicBreaker;
        import OmniPES.Units;
        Modelica.Blocks.Interfaces.BooleanInput ext_open annotation(
          Placement(visible = true, transformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 88}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      equation
        ext_open = open;
        annotation(
          Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {0.5, 0.5}), graphics = {Line(origin = {0, 53}, points = {{0, 17}, {0, -29}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled})}),
          Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
      end Breaker;

      model TimedBreaker
        extends OmniPES.Circuit.Switches.Interfaces.BasicBreaker;
        import OmniPES.Units;
        parameter Real t_open;
      equation
        open = if time >= t_open then true else false;
        annotation(
          Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {0.5, 0.5}), graphics = {Text(origin = {-1, -59}, extent = {{-99, 39}, {99, -39}}, textString = "%t_open [s]")}),
          Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
      end TimedBreaker;

      model Fault
        import OmniPES.Units;
        import Modelica.Units.SI;
        parameter Units.PerUnit R = 0;
        parameter Units.PerUnit X = 1e-3;
        parameter SI.Time t_on = 0.1;
        parameter SI.Time t_off = 0.2;
        OmniPES.Circuit.Interfaces.PositivePin T annotation(
          Placement(visible = true, transformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-1.77636e-15, 100}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
        Boolean closed(start = false);
      protected
        OmniPES.Circuit.Switches.Breaker breaker annotation(
          Placement(visible = true, transformation(origin = {0, 76}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Basic.ShuntAdmittance shuntAdmittance(g = R / (R ^ 2 + X ^ 2), b = -X / (R ^ 2 + X ^ 2)) annotation(
          Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
        breaker.open = not closed;
        closed = if time >= t_on and time < t_off then true else false;
        connect(breaker.p, T) annotation(
          Line(points = {{0, 86}, {0, 110}}, color = {0, 0, 255}));
        connect(breaker.n, shuntAdmittance.p) annotation(
          Line(points = {{0, 66}, {0, 50}}, color = {0, 0, 255}));
      protected
        annotation(
          Icon(coordinateSystem(grid = {0.5, 0.5}, initialScale = 0.1), graphics = {Line(origin = {10, 60}, points = {{-20, 0}, {20, 0}}), Line(origin = {-20, 44}, points = {{10, 16}, {-10, -16}}), Line(origin = {20, 44}, points = {{10, 16}, {-10, -16}}), Line(origin = {20, 12}, points = {{10, 16}, {-10, -16}}), Line(origin = {-20, 12}, points = {{10, 16}, {-10, -16}, {-10, -16}}), Line(origin = {-20, -20}, points = {{10, 16}, {-10, -40}}), Line(origin = {-20, 28}, points = {{-10, 0}, {10, 0}}), Line(origin = {20, 28}, points = {{-10, 0}, {10, 0}}), Line(origin = {-20, -4}, points = {{-10, 0}, {10, 0}}), Line(origin = {12, -20}, points = {{10, 16}, {-42, -40}}), Line(origin = {16, -4}, points = {{-6, 0}, {6, 0}}), Rectangle(origin = {-15, -1}, extent = {{-45, 71}, {75, -71}}), Line(origin = {0, 86}, points = {{0, 16}, {0, -16}})}),
          Diagram(coordinateSystem(grid = {0.5, 0.5})),
          __OpenModelica_commandLineOptions = "");
      end Fault;

      package Interfaces
        extends Modelica.Icons.InterfacesPackage;

        partial model BasicBreaker
          extends OmniPES.Circuit.Interfaces.SeriesComponent;
          import OmniPES.Units;
          Boolean open(start = false);
          parameter Real Ron(final min = 0) = 1e-6;
          parameter Real Goff(final min = 0) = 1e-6;
          Complex s(re(start = 1)) "Auxiliary variable";
        equation
//if open then
//  i = Complex(0,0);
//else
//  v = Complex(0,0);
//end if;
//  when open then
//     i := Complex(0);
//     v := s;
//   elsewhen not open then
//     v := Complex(0);
//     i := s;
//   end when;
          v = s * (if open then 1 else Ron);
          i = s * (if open then Goff else 1);
          annotation(
            Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {0.5, 0.5}), graphics = {Line(origin = {-70, 0}, points = {{-30, 0}, {30, 0}}, thickness = 1), Line(origin = {70, 0}, points = {{-30, 0}, {30, 0}}, thickness = 1), Line(origin = {-40, 0}, points = {{0, 0}, {69.28, 40}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {50, 0}, points = {{-10, 10}, {10, -10}, {10, -10}}), Line(origin = {50, 0}, points = {{-10, -10}, {10, 10}, {10, 10}})}),
            Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})));
        end BasicBreaker;
      end Interfaces;
      annotation(
        Icon(coordinateSystem(initialScale = 0.1, grid = {0.5, 0.5}), graphics = {Line(origin = {-70, 0}, points = {{-10, 0}, {30, 0}}, thickness = 1), Line(origin = {70, 0}, points = {{-20, 0}, {10, 0}}, thickness = 1), Line(origin = {-40, 0}, points = {{0, 0}, {69.28, 40}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 6), Line(origin = {50, 0}, points = {{-10, 10}, {10, -10}, {10, -10}}), Line(origin = {50, 0}, points = {{-10, -10}, {10, 10}, {10, 10}}), Rectangle(lineThickness = 1, extent = {{-100, 100}, {100, -100}}), Ellipse(origin = {-80, 0}, fillPattern = FillPattern.Solid, extent = {{2, 2}, {-2, -2}}, endAngle = 360), Ellipse(origin = {80, 0}, fillPattern = FillPattern.Solid, extent = {{2, 2}, {-2, -2}}, endAngle = 360)}),
        Diagram(coordinateSystem(extent = {{-100, -150}, {100, 100}})));
    end Switches;
  end Circuit;

  package WindTurbine
    model DFIG_WT
      // Recebendo dados por parâmetros externo:
      parameter WindTurbine.Interfaces.DWTData smData;
      Modelica.Blocks.Interfaces.RealInput VW annotation(
        Placement(visible = true, transformation(origin = {-100, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Qref annotation(
        Placement(visible = true, transformation(origin = {-4, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      OmniPES.Circuit.Interfaces.PositivePin pin_WT annotation(
        Placement(visible = true, transformation(origin = {96, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      OmniPES.WindTurbine.SENSORS.PowerSensor powerSensor annotation(
        Placement(visible = true, transformation(origin = {68, 36}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      OmniPES.WindTurbine.CONVERSOR_model.CONVERSOR conversor(Ceq = smData.convData.Ceq, Di = smData.convData.Di) annotation(
        Placement(visible = true, transformation(origin = {8, 12}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor tac annotation(
        Placement(visible = true, transformation(origin = {-51, 31}, extent = {{-5, -5}, {5, 5}}, rotation = -90)));
      OmniPES.WindTurbine.DFIG_model.MIT_fm mIT_fm(Dm = smData.convData.Dm, Hm = smData.convData.Hm, Llr = smData.convData.Llr, Lls = smData.convData.Lls, Lm = smData.convData.Lm, rr = smData.convData.rr, rs = smData.convData.rs) annotation(
        Placement(visible = true, transformation(origin = {-30, 36}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
      OmniPES.WindTurbine.SENSORS.VoltageSensor voltageSensor annotation(
        Placement(visible = true, transformation(origin = {76, -26}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      OmniPES.WindTurbine.TURBINA_model.EOLICA eolica(Dtm = smData.convData.Dtm, Dtur = smData.convData.Dt, Htur = smData.convData.Ht, Ktm = smData.convData.Ktm, N = smData.N, R = smData.R, Sb = smData.MVAs * 1e6, Wrmb = smData.Wrmb, par = smData.par, ts = smData.ts_beta, zeta = smData.zeta_beta) annotation(
        Placement(visible = true, transformation(origin = {-68, 36}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
      OmniPES.WindTurbine.CONTROL_model.CTRL_MAQ ctrl_maq(Ceq = smData.convData.Ceq, Dtotal = smData.convData.Dm + smData.convData.Dt, Htotal = smData.convData.Hm + smData.convData.Ht, Lm = smData.convData.Lm, Ls = smData.convData.Lls + smData.convData.Lm, Vccref = smData.Vccref, kiPLL = smData.kiPLL, kiQs = smData.kiQs, kpPLL = smData.kpPLL, tsVcc = smData.tsVcc, tsWrm = smData.tsWrm, zetaVcc = smData.zetaVcc, zetaWrm = smData.zetaWrm) annotation(
        Placement(visible = true, transformation(origin = {8, -28}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      OmniPES.WindTurbine.CONTROL_model.CRTL_TUR crtl_tur(LBD_opt = smData.LBD_opt, N = smData.N, R = smData.R, Vw_max = smData.Vw_max, Vw_min = smData.Vw_min, Vw_nom = smData.Vw_nom, Vw_wmax = smData.Vw_wmax, Vw_wmin = smData.Vw_wmin, Wrmb = smData.Wrmb) annotation(
        Placement(visible = true, transformation(origin = {-72, -32}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      OmniPES.Circuit.Basic.SeriesImpedance seriesImpedance1(r = 0, x = smData.convData.Lcg) annotation(
        Placement(visible = true, transformation(origin = {44, 12}, extent = {{-6, -6}, {6, 6}}, rotation = 180)));
    equation
      connect(tac.flange, mIT_fm.eixo) annotation(
        Line(points = {{-51, 36}, {-30, 36}}));
      connect(eolica.flange_b, tac.flange) annotation(
        Line(points = {{-59.6, 36}, {-50.6, 36}}));
      connect(VW, eolica.Vw) annotation(
        Line(points = {{-100, 36}, {-74, 36}}, color = {0, 0, 127}));
      connect(conversor.outVcc, ctrl_maq.Vccmed) annotation(
        Line(points = {{8, 0.16}, {8, -9.84}}, color = {0, 0, 127}));
      connect(ctrl_maq.Wmed, tac.w) annotation(
        Line(points = {{-14, -20}, {-51, -20}, {-51, 25.5}}, color = {0, 0, 127}));
      connect(ctrl_maq.Vqds, voltageSensor.outV) annotation(
        Line(points = {{30, -26}, {71, -26}}, color = {0, 0, 127}, thickness = 0.5));
      connect(eolica.Beta, crtl_tur.beta) annotation(
        Line(points = {{-72.2, 24.8}, {-72, 24.8}, {-72, -18}}, color = {0, 0, 127}));
      connect(powerSensor.outS[2], ctrl_maq.Qmed) annotation(
        Line(points = {{68, 31}, {68, -20.4}, {30, -20.4}}, color = {0, 0, 127}));
      connect(VW, crtl_tur.Vw) annotation(
        Line(points = {{-100, 36}, {-90, 36}, {-90, -26}}, color = {0, 0, 127}));
      connect(crtl_tur.Wrm_opt, ctrl_maq.Wref) annotation(
        Line(points = {{-54.4, -25.6}, {-14.4, -25.6}}, color = {0, 0, 127}));
      connect(Qref, ctrl_maq.Qref) annotation(
        Line(points = {{-4, -50}, {8, -50}, {8, -30}}, color = {0, 0, 127}));
      connect(mIT_fm.pin_p, powerSensor.pin_p) annotation(
        Line(points = {{-14, 36}, {62, 36}}, color = {0, 0, 255}));
      connect(powerSensor.pin_n, pin_WT) annotation(
        Line(points = {{74, 36}, {96, 36}}, color = {0, 0, 255}));
      connect(pin_WT, voltageSensor.pin_p) annotation(
        Line(points = {{96, 36}, {76, 36}, {76, -20}}, color = {0, 0, 255}));
      connect(seriesImpedance1.p, powerSensor.pin_p) annotation(
        Line(points = {{50, 12}, {62, 12}, {62, 36}}, color = {0, 0, 255}));
      connect(ctrl_maq.Mqdr, conversor.Ir) annotation(
        Line(points = {{-2, -10}, {-2, -2}}, color = {85, 170, 255}));
      connect(ctrl_maq.Mqdg, conversor.Ig) annotation(
        Line(points = {{18, -10}, {18, -2}}, color = {85, 170, 255}));
      connect(conversor.outR, mIT_fm.pin_n) annotation(
        Line(points = {{-10, 12}, {-30, 12}, {-30, 28}}, color = {0, 0, 255}));
      connect(conversor.outG, seriesImpedance1.n) annotation(
        Line(points = {{26, 12}, {38, 12}}, color = {0, 0, 255}));
    protected
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, lineColor = {0, 0, 255}, extent = {{-65, 52}, {65, -52}}, textString = "DFIG")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end DFIG_WT;

    package Interfaces
      extends Modelica.Icons.InterfacesPackage;

      record DWTData
        extends Modelica.Icons.Record;
        import OmniPES.Units;
        import SI = Modelica.Units.SI;
        parameter Units.ApparentPower MVAs = 2 "System base power" annotation(
          Dialog(group = "Base Quatities"));
        parameter Units.ApparentPower MVAb = 2 "Machine base power" annotation(
          Dialog(group = "Base Quatities"));
        parameter Integer Nmaq = 1 "Number of parallel machines" annotation(
          Dialog(group = "Base Quatities"));
        parameter Real Wb = 1 "Frequency base" annotation(
          Dialog(group = "Base Quatities"));
        parameter Real Wrmb = 2 * Wb / Polos "Frequency mech base" annotation(
          Dialog(group = "Base Calculed"));
        // Elec data:
        parameter Units.PerUnit rs = 0.01 "Stator resistance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit rr = 0.01 "Rotor resistance" annotation(
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
        parameter Units.PerUnit Vw_wmin = 8.071353643090237 "Min wind for rotor speed" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit Vw_wmax = 11.175720428894175 "Max wind for rotor speed" annotation(
          Dialog(group = "Turbine Data"));
        parameter Units.PerUnit LBD_opt = 6.3279 "Tip speed ratio optimal" annotation(
          Dialog(group = "Turbine Data"));
        // Conv data:
        parameter Units.PerUnit Di = 100 "Degenerator impedance" annotation(
          Dialog(group = "Conversor Data"));
        parameter Units.PerUnit Ceq = 35.897 "Capacitor of converter" annotation(
          Dialog(group = "Conversor Data"));
        parameter Units.PerUnit Lcg = 0.040 "Inductance grid side" annotation(
          Dialog(group = "Conversor Data"));
        // Tranformer data
        parameter Units.PerUnit Lct = 0.000 "Inductance transformer interface" annotation(
          Dialog(group = "Transformer Data"));
        // CTRL pitch data:
        parameter SI.Time ts_beta = 1e-2 "Settiling time by pitch angle" annotation(
          Dialog(group = "Pitch Control Data"));
        parameter Units.PerUnit zeta_beta = 1 "Damping constant by pitch angle" annotation(
          Dialog(group = "Pitch Control Data"));
        // CTRL Wrm data:
        parameter SI.Time tsWrm = 1 "Settiling time by speed" annotation(
          Dialog(group = "Velocity Control Data"));
        parameter Units.PerUnit zetaWrm = 1 "Damping constant by speed" annotation(
          Dialog(group = "Velocity Control Data"));
        // CTRL Qs data:
        parameter Units.PerUnit kiQs = 1 "Integral constant by reactivepower" annotation(
          Dialog(group = "Reactive Power Control Data"));
        // CTRL PLL data:
        parameter Units.PerUnit kiPLL = 20 "Integral constant by PLL" annotation(
          Dialog(group = "PLL Control Data"));
        parameter Units.PerUnit kpPLL = 2 "Proporcional constant by PLL" annotation(
          Dialog(group = "PLL Control Data"));
        // CTRL Vcc data:
        parameter Units.PerUnit Vccref = 1 "Reference voltage by Vcc" annotation(
          Dialog(group = "GSC Control Data"));
        parameter Units.PerUnit zetaVcc = 1 "Damping constant by Vcc" annotation(
          Dialog(group = "GSC Control Data"));
        parameter SI.Time tsVcc = 3 "Settiling time by Vcc" annotation(
          Dialog(group = "GSC Control Data"));

        record ConvertedData
          parameter Units.PerUnit rs = 0.01 annotation(
            Dialog(group = "Electrical Data"));
          parameter Units.PerUnit rr = 0.01 annotation(
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
          parameter Units.PerUnit Dm = 0.0 annotation(
            Dialog(group = "Mechanical Data"));
          parameter Units.PerUnit Ht = 4.2 annotation(
            Dialog(group = "Mechanical Data"));
          parameter Units.PerUnit Dt = 0.0 annotation(
            Dialog(group = "Mechanical Data"));
          parameter Units.PerUnit Ktm = 0.3 annotation(
            Dialog(group = "Mechanical Data"));
          parameter Units.PerUnit Dtm = 1.5 annotation(
            Dialog(group = "Mechanical Data"));
          // Conversor:
          parameter Units.PerUnit Di = 100 "Degenerator admitance" annotation(
            Dialog(group = "Conversor Data"));
          parameter Units.PerUnit Ceq = 35.897 annotation(
            Dialog(group = "Conversor Data"));
          parameter Units.PerUnit Lcg = 0.040 annotation(
            Dialog(group = "Conversor Data"));
          // Transformer:
          parameter Units.PerUnit Lct = 0.000 annotation(
            Dialog(group = "Transformer Data"));
        end ConvertedData;

        ConvertedData convData(rs = MVAs / MVAb / Nmaq * rs, rr = MVAs / MVAb / Nmaq * rr, Di = MVAb * Nmaq / MVAs * Di, Lls = MVAs / MVAb / Nmaq * Lls, Llr = MVAs / MVAb / Nmaq * Llr, Lm = MVAs / MVAb / Nmaq * Lm, Hm = Nmaq * MVAb / MVAs * Hm, Dm = Nmaq * MVAb / MVAs * Dm, Ht = Nmaq * MVAb / MVAs * Ht, Dt = Nmaq * MVAb / MVAs * Dt, Ktm = Nmaq * MVAb / MVAs * Ktm, Dtm = Nmaq * MVAb / MVAs * Dtm, Ceq = MVAs / MVAb / Nmaq * Ceq, Lct = MVAs / MVAb / Nmaq * Lct, Lcg = MVAs / MVAb / Nmaq * Lcg);
        annotation(
          defaultComponentName = "smData",
          defaultVariability = "Parameter");
      end DWTData;
    end Interfaces;

    package TURBINA_model
      extends Modelica.Icons.Package;

      function CP
        input Real Wtur;
        input Real Vw;
        input Real beta "Pitch angle in degree";
        output Real Cp "Power coefficient";
      protected
        Real lambda, alpha;
        constant Real R = 37.5;
        constant Real C1 = 0.22;
        constant Real C2 = 116;
        constant Real C3 = 0.4;
        constant Real C4 = 0;
        constant Real C5 = 0;
        constant Real C6 = 5;
        constant Real C7 = 12.5;
        constant Real C8 = 0.08;
        constant Real C9 = 0.035;
      algorithm
        lambda := R * Wtur / Vw;
        alpha := 1 / (1 / (lambda + beta * C8) - C9 / (beta ^ 3 + 1));
        Cp := C1 * (C2 / alpha - C3 * beta - C4 * beta ^ C5 - C6) * exp(-C7 / alpha);
      end CP;

      model TURBINA
        import SI = Modelica.Units.SI;
        import pi = Modelica.Constants.pi;
        // Base values:
        parameter Real Sb = 2e6 annotation(
          Dialog(group = "Base Data"));
        parameter Real Wrmb = 60 * pi annotation(
          Dialog(group = "Base Data"));
        // Mechanical parameters:
        parameter OmniPES.Units.PerUnit Dtur = 0 annotation(
          Dialog(group = "Mechanical Data"));
        parameter SI.Time Htur = 4.2 annotation(
          Dialog(group = "Mechanical Data"));
        // Aerodynamic parameters:
        parameter Real par = 1.225 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real R = 37.5 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real A = pi * R ^ 2 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real N = 111.5 annotation(
          Dialog(group = "Turbine Data"));
        // parameteres de controle do beta:
        parameter Real ts = 0.01 annotation(
          Dialog(group = "Control Data"));
        parameter Real zeta = 0.7 annotation(
          Dialog(group = "Control Data"));
        // Determiando ganhos:
        parameter Real kb = 2 / ts annotation(
          Dialog(group = "Control Calculed"));
        parameter Real tb = ts / (8 * zeta ^ 2) annotation(
          Dialog(group = "Control Calculed"));
        // Declaração de variáveis:
        OmniPES.Units.PerUnit Ttur, Teixo, Ptur, Wtur, Wcp;
        Real cp "%";
        // Conexões de interface:
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_Eixo annotation(
          Placement(visible = true, transformation(origin = {104, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vw annotation(
          Placement(visible = true, transformation(origin = {-118, 48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Beta annotation(
          Placement(visible = true, transformation(origin = {-118, -58}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Continuous.TransferFunction tfBeta(a = {tb, 1, kb}, b = {kb}, initType = Modelica.Blocks.Types.Init.SteadyState) annotation(
          Placement(visible = true, transformation(origin = {-76, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      initial equation
        der(Wtur) = 0;
      equation
        Wcp = Wtur * Wrmb / N;
        Teixo = flange_Eixo.tau;
        Wtur = der(flange_Eixo.phi);
// Convertendo velocidade em conjugado:
        cp = CP(Wcp, Vw, tfBeta.y);
        Ptur = 1 / 2 * (par * A * cp * Vw ^ 3) / Sb;
        Ttur = Ptur / Wtur;
// Equações mecânicas:
        2 * Htur * der(Wtur) = Ttur + Teixo - Dtur * Wtur;
        connect(Beta, tfBeta.u) annotation(
          Line(points = {{-118, -58}, {-88, -58}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Ellipse(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{22, -56}, {22, -56}}), Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {1, -1}, lineColor = {0, 0, 255}, extent = {{-67, 29}, {67, -29}}, textString = "TURBINA")}),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
          Diagram);
      end TURBINA;

      model EOLICA
        //######################################### PARAMETERS:
        import SI = Modelica.Units.SI;
        import pi = Modelica.Constants.pi;
        //  Base values:
        parameter Real Sb = 2e6 annotation(
          Dialog(group = "Base Data"));
        parameter Real Wrmb = 60 * pi annotation(
          Dialog(group = "Base Data"));
        // Mechanical parameters:
        parameter OmniPES.Units.PerUnit Dtur = 0 annotation(
          Dialog(group = "Mechanical Data"));
        parameter SI.Time Htur = 4.2 annotation(
          Dialog(group = "Mechanical Data"));
        parameter OmniPES.Units.PerUnit Dtm = 1.5 annotation(
          Dialog(group = "Mechanical Data"));
        parameter OmniPES.Units.PerUnit Ktm = 0.3 annotation(
          Dialog(group = "Mechanical Data"));
        // Aerodynamic parameters:
        parameter Real par = 1.225 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real R = 37.5 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real A = pi * R ^ 2 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real N = 111.5 annotation(
          Dialog(group = "Turbine Data"));
        // parameteres de controle do beta:
        parameter Real ts = 0.01 annotation(
          Dialog(group = "Control Data"));
        parameter Real zeta = 0.7 annotation(
          Dialog(group = "Control Data"));
        // Determiando ganhos:
        parameter Real kb = 2 / ts annotation(
          Dialog(group = "Control Calculed"));
        parameter Real tb = ts / (8 * zeta ^ 2) annotation(
          Dialog(group = "Control Calculed"));
        //#########################################
        OmniPES.WindTurbine.TURBINA_model.TURBINA turbina(A = A, Dtur = Dtur, Htur = Htur, N = N, R = R, Sb = Sb, Wrmb = Wrmb, par = par, ts = ts, zeta = zeta) annotation(
          Placement(visible = true, transformation(origin = {-20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vw annotation(
          Placement(visible = true, transformation(origin = {-100, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-42, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Beta annotation(
          Placement(visible = true, transformation(origin = {-20, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-30, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation(
          Placement(visible = true, transformation(origin = {104, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c = Ktm, d = Dtm, phi_rel0(displayUnit = "mrad")) annotation(
          Placement(visible = true, transformation(origin = {20, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      initial equation
        der(springDamper.flange_a.phi - springDamper.flange_b.phi) = 0;
      equation
        connect(Vw, turbina.Vw) annotation(
          Line(points = {{-100, 40}, {-31, 40}}, color = {0, 0, 127}));
        connect(Beta, turbina.Beta) annotation(
          Line(points = {{-20, -2}, {-20, 30}}, color = {0, 0, 127}));
        connect(springDamper.flange_a, turbina.flange_Eixo) annotation(
          Line(points = {{10, 40}, {-9, 40}}));
        connect(springDamper.flange_b, flange_b) annotation(
          Line(points = {{30, 40}, {104, 40}}));
        annotation(
          Icon(graphics = {Rectangle(origin = {35, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-15, 6}, {15, -6}}), Rectangle(origin = {3, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Solid, extent = {{-17, 20}, {17, -20}}), Ellipse(origin = {-10, 0}, lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-20, 20}, {20, -20}}), Polygon(origin = {0, 80}, lineColor = {0, 0, 255}, points = {{0, -60}, {20, -60}, {20, 120}, {-20, 0}, {-20, 0}, {0, -60}}), Polygon(origin = {0, -110}, lineColor = {0, 0, 255}, points = {{20, 90}, {20, -90}, {-20, 30}, {2, 90}, {2, 90}, {20, 90}})}));
      end EOLICA;
    end TURBINA_model;

    package DFIG_model
      extends Modelica.Icons.Package;

      model MIT_fm
        //  Importando bibliotecas necessárias:
        import SI = Modelica.Units.SI;
        import pi = Modelica.parameters.pi;
        import cm = Modelica.ComplexMath;
        import j = Modelica.ComplexMath.j;
        import MechInterface = Modelica.Mechanics.Rotational.Interfaces;
        import absC = Modelica.ComplexMath.abs;
        //  Declarando variáveis do problema:
        OmniPES.Units.PerUnit theta_rm;
        OmniPES.Units.PerUnit Wr, Wrm;
        OmniPES.Units.PerUnit Vqs, Vds, Vqr, Vdr;
        OmniPES.Units.PerUnit Iqs, Ids, Iqr, Idr;
        OmniPES.Units.PerUnit fqs, fds, fqr, fdr;
        OmniPES.Units.PerUnit Te, Tm;
        OmniPES.Units.CPerUnit Ss, Sr, Ssr;
        OmniPES.Units.PerUnit Sc, Ir;
        //  Parâmetros MIT 2 MW, 690V, 60Hz (pu):
        constant OmniPES.Units.PerUnit We = 1;
        parameter OmniPES.Units.PerUnit rs = 0.01 annotation(
          Dialog(group = "Eletrical Data"));
        parameter OmniPES.Units.PerUnit rr = 0.01 annotation(
          Dialog(group = "Eletrical Data"));
        parameter OmniPES.Units.PerUnit Lls = 0.1 annotation(
          Dialog(group = "Eletrical Data"));
        parameter OmniPES.Units.PerUnit Llr = 0.08 annotation(
          Dialog(group = "Eletrical Data"));
        parameter OmniPES.Units.PerUnit Lm = 3.0 annotation(
          Dialog(group = "Eletrical Data"));
        parameter OmniPES.Units.PerUnit Dm = 0 annotation(
          Dialog(group = "Mechanical Data"));
        parameter SI.Time Hm = 0.524 annotation(
          Dialog(group = "Mechanical Data"));
        // Interfaces:
        Modelica.Mechanics.Rotational.Interfaces.Flange_a eixo annotation(
          Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.PositivePin pin_p annotation(
          Placement(visible = true, transformation(origin = {-6, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin pin_n annotation(
          Placement(visible = true, transformation(origin = {8, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      initial equation
        theta_rm = 0;
        der(Wrm) = 0;
        der(fqs) = 0;
        der(fds) = 0;
        der(fqr) = 0;
        der(fdr) = 0;
      equation
// Magnitude do fasor corrente do rotor:
        Ir ^ 2 = Iqr ^ 2 + Idr ^ 2;
//  Entradas externas:
        Vqs = pin_p.v.re;
        Vds = pin_p.v.im;
        Iqs = pin_p.i.re;
        Ids = pin_p.i.im;
        Vqr = pin_n.v.re;
        Vdr = pin_n.v.im;
        Iqr = pin_n.i.re;
        Idr = pin_n.i.im;
//  Conexões de interface:
        Tm = eixo.tau;
        theta_rm = eixo.phi;
        der(theta_rm) = Wrm;
//  Estator Fluxo e Tensão:
        fqs = Lls * Iqs + Lm * (Iqs + Iqr);
        fds = Lls * Ids + Lm * (Ids + Idr);
        Vqs = rs * Iqs + We * fds + 0*der(fqs);
        Vds = rs * Ids - We * fqs + 0*der(fds);
//  Rotor Fluxo e Tensão:
        fqr = Llr * Iqr + Lm * (Iqs + Iqr);
        fdr = Llr * Idr + Lm * (Ids + Idr);
        Vqr = rr * Iqr + (We - Wr) * fdr + der(fqr);
        Vdr = rr * Idr - (We - Wr) * fqr + der(fdr);
//  Potências:
        Ss = (Vqs - j * Vds) * (Iqs + j * Ids);
        Sr = (Vqr - j * Vdr) * (Iqr + j * Idr);
        Ssr = Ss + Sr;
        Sc = absC(Sr);
//  Conjugados:
        Te = fds * Iqs - fqs * Ids;
//  Modelo Mecânico:
        2 * Hm * der(Wrm) = Tm + Te - Dm * Wrm;
        Wr = Wrm;
        annotation(
          experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.0001),
          Diagram(coordinateSystem(extent = {{-20, 20}, {20, -20}})),
          Icon(graphics = {Bitmap(extent = {{20, 0}, {20, 0}}), Ellipse(lineColor = {0, 0, 255}, fillColor = {0, 0, 255}, extent = {{-100, 100}, {100, -100}}), Ellipse(lineColor = {0, 0, 255}, extent = {{-50, 50}, {50, -50}})}));
      end MIT_fm;
    end DFIG_model;

    package CONVERSOR_model
      extends Modelica.Icons.Package;

      model CONVERSOR
        // Parametros conversor:
        parameter Units.PerUnit Di = 100 "Degenerator admitance" annotation(
          Dialog(group = "Conversor Data"));
        parameter Units.PerUnit Ceq = 35.897 "Capacitor of converter" annotation(
          Dialog(group = "Conversor Data"));
        OmniPES.Units.PerUnit Pcap, Pcc_RSC, Pcc_GSC;
        OmniPES.Circuit.Interfaces.PositivePin outR annotation(
          Placement(visible = true, transformation(origin = {-120, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-114, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.ComplexBlocks.Interfaces.ComplexInput Ir annotation(
          Placement(visible = true, transformation(origin = {-70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-60, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.ComplexBlocks.Interfaces.ComplexInput Ig annotation(
          Placement(visible = true, transformation(origin = {60, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {60, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealOutput outVcc annotation(
          Placement(visible = true, transformation(origin = {-26, -30}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -74}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Interfaces.PositivePin outG annotation(
          Placement(visible = true, transformation(origin = {120, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {114, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Electrical.Analog.Basic.Ground ground annotation(
          Placement(visible = true, transformation(origin = {14, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Electrical.Analog.Basic.Capacitor capacitor(C = Ceq) annotation(
          Placement(visible = true, transformation(origin = {14, 14}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Electrical.Analog.Sources.SignalCurrent IccRSC annotation(
          Placement(visible = true, transformation(origin = {-46, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Electrical.Analog.Sources.SignalCurrent IccGSC annotation(
          Placement(visible = true, transformation(origin = {34, 14}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
        Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(
          Placement(visible = true, transformation(origin = {-14, 14}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Sources.ControlledCurrentSource currRSC annotation(
          Placement(visible = true, transformation(origin = {-82, 8}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Sources.ControlledCurrentSource currGSC annotation(
          Placement(visible = true, transformation(origin = {78, 6}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
        Circuit.Basic.Ground ground1 annotation(
          Placement(visible = true, transformation(origin = {-100, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Circuit.Basic.SeriesImpedance degImpR(r = Di) annotation(
          Placement(visible = true, transformation(origin = {-100, 4}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Circuit.Basic.Ground ground2 annotation(
          Placement(visible = true, transformation(origin = {108, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Circuit.Basic.SeriesImpedance degImpG(r = Di) annotation(
          Placement(visible = true, transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      initial equation
        der(capacitor.v) = 0;
      equation
        Pcap = capacitor.v * capacitor.i;
        Pcc_RSC = IccRSC.i * capacitor.v;
        Pcc_GSC = IccGSC.i * capacitor.v;
        Pcc_RSC = currRSC.S.re;
        Pcc_GSC = currGSC.S.re;
        connect(ground.p, capacitor.n) annotation(
          Line(points = {{14, 0}, {14, 4}}, color = {0, 0, 255}));
        connect(voltageSensor.p, capacitor.p) annotation(
          Line(points = {{-14, 24}, {14, 24}}, color = {0, 0, 255}));
        connect(ground.p, voltageSensor.n) annotation(
          Line(points = {{14, 0}, {-14, 0}, {-14, 4}}, color = {0, 0, 255}));
        connect(voltageSensor.v, outVcc) annotation(
          Line(points = {{-25, 14}, {-26, 14}, {-26, -30}}, color = {0, 0, 127}));
        connect(ground.p, IccRSC.p) annotation(
          Line(points = {{14, 0}, {-46, 0}, {-46, 4}}, color = {0, 0, 255}));
        connect(ground.p, IccGSC.p) annotation(
          Line(points = {{14, 0}, {34, 0}, {34, 4}}, color = {0, 0, 255}));
        connect(voltageSensor.p, IccRSC.n) annotation(
          Line(points = {{-14, 24}, {-46, 24}}, color = {0, 0, 255}));
        connect(IccGSC.n, capacitor.p) annotation(
          Line(points = {{34, 24}, {14, 24}}, color = {0, 0, 255}));
        connect(currRSC.p, outR) annotation(
          Line(points = {{-82, 18}, {-120, 18}}, color = {0, 0, 255}));
        connect(currGSC.p, outG) annotation(
          Line(points = {{78, 16}, {120, 16}}, color = {0, 0, 255}));
        connect(Ig, currGSC.u) annotation(
          Line(points = {{60, -30}, {60, 6}, {70, 6}}, color = {85, 170, 255}));
        connect(currRSC.u, Ir) annotation(
          Line(points = {{-74, 8}, {-70, 8}, {-70, -30}}, color = {85, 170, 255}));
        connect(ground1.p, degImpR.n) annotation(
          Line(points = {{-100, -12}, {-100, -6}}, color = {0, 0, 255}));
        connect(degImpR.p, currRSC.p) annotation(
          Line(points = {{-100, 14}, {-100, 18}, {-82, 18}}, color = {0, 0, 255}));
        connect(ground2.p, degImpG.n) annotation(
          Line(points = {{108, -16}, {108, -10}}, color = {0, 0, 255}));
        connect(outG, degImpG.p) annotation(
          Line(points = {{120, 16}, {108, 16}, {108, 10}}, color = {0, 0, 255}));
        annotation(
          Icon(graphics = {Rectangle(origin = {-60, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-40, 80}, {40, -80}}), Rectangle(origin = {60, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-40, 80}, {40, -80}}), Line(origin = {-0.5, 35}, points = {{-19.5, 25}, {20.5, 25}, {20.5, 25}, {0.5, 25}, {0.5, -25}, {-9.5, -25}, {10.5, -25}, {10.5, -25}}, color = {0, 0, 255}), Line(origin = {-0.5, -35}, points = {{-19.5, -25}, {20.5, -25}, {0.5, -25}, {0.5, 35}, {-9.5, 35}, {10.5, 35}, {10.5, 35}}, color = {0, 0, 255}), Line(origin = {-85.43, -0.5}, points = {{-7, 0.5}, {7, 0.5}, {7, 20.5}, {7, -19.5}, {7, -19.5}}, color = {0, 0, 255}), Line(origin = {-61.77, -0.02}, points = {{-8.66454, 20.0191}, {-8.66454, -19.9809}, {-8.66454, 0.019071}, {-8.66454, 10.0191}, {7.33546, 40.0191}, {-8.66454, 10.0191}, {-8.66454, -9.9809}, {9.33546, -39.9809}, {-8.66454, -9.9809}, {-8.66454, -9.9809}}, color = {0, 0, 255}), Line(origin = {-41.16, -3.28}, points = {{-17.9642, 34}, {8.03576, 34}, {8.03576, 6}, {4.03576, 6}, {8.03576, -4}, {12.0358, 6}, {8.03576, 6}, {12.0358, 6}, {8.03576, -4}, {4.03576, -4}, {12.0358, -4}, {8.03576, -4}, {8.03576, -28}, {-15.9642, -28}, {-15.9642, -28}}, color = {0, 0, 255}), Line(origin = {76.44, -3.28}, points = {{-17.9642, 34}, {8.03576, 34}, {8.03576, 6}, {4.03576, 6}, {8.03576, -4}, {12.0358, 6}, {8.03576, 6}, {12.0358, 6}, {8.03576, -4}, {4.03576, -4}, {12.0358, -4}, {8.03576, -4}, {8.03576, -28}, {-15.9642, -28}, {-15.9642, -28}}, color = {0, 0, 255}), Line(origin = {55.83, -0.02}, points = {{-8.66454, 20.0191}, {-8.66454, -19.9809}, {-8.66454, 0.019071}, {-8.66454, 10.0191}, {7.33546, 40.0191}, {-8.66454, 10.0191}, {-8.66454, -9.9809}, {9.33546, -39.9809}, {-8.66454, -9.9809}, {-8.66454, -9.9809}}, color = {0, 0, 255}), Line(origin = {32.17, -0.5}, points = {{-7, 0.5}, {7, 0.5}, {7, 20.5}, {7, -19.5}, {7, -19.5}}, color = {0, 0, 255})}),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
      end CONVERSOR;
    end CONVERSOR_model;

    package CONTROL_model
      extends Modelica.Icons.Package;

      model PARK
        Modelica.Blocks.Interfaces.RealInput u[2] annotation(
          Placement(visible = true, transformation(origin = {32, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
          Placement(visible = true, transformation(origin = {-98, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput theta annotation(
          Placement(visible = true, transformation(origin = {-22, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      equation
        y[1] = cos(theta) * u[1] + sin(theta) * u[2];
        y[2] = (-sin(theta) * u[1]) + cos(theta) * u[2];
        annotation(
          Diagram,
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {3, -1}, lineColor = {0, 0, 255}, extent = {{-43, 33}, {43, -33}}, textString = "PARK")}));
      end PARK;

      model iPARK
        Modelica.Blocks.Interfaces.RealInput u[2] annotation(
          Placement(visible = true, transformation(origin = {32, 10}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealOutput y[2] annotation(
          Placement(visible = true, transformation(origin = {-98, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput theta annotation(
          Placement(visible = true, transformation(origin = {-8, -94}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      equation
        y[1] = cos(theta) * u[1] - sin(theta) * u[2];
        y[2] = sin(theta) * u[1] + cos(theta) * u[2];
        annotation(
          Diagram,
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {3, -1}, lineColor = {0, 0, 255}, extent = {{-43, 33}, {43, -33}}, textString = "PARK⁻¹")}));
      end iPARK;

      model PLL
        parameter Real kp = 10, ki = 10;
        OmniPES.WindTurbine.CONTROL_model.PARK park annotation(
          Placement(visible = true, transformation(origin = {40, -3.55271e-15}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
        Modelica.Blocks.Continuous.PI pi(T = kp / ki, initType = Modelica.Blocks.Types.Init.SteadyState, k = kp) annotation(
          Placement(visible = true, transformation(origin = {0, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.NoInit) annotation(
          Placement(visible = true, transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput Phi annotation(
          Placement(visible = true, transformation(origin = {-106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealOutput Vqds[2] annotation(
          Placement(visible = true, transformation(origin = {-104, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput u[2] annotation(
          Placement(visible = true, transformation(origin = {108, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      equation
        connect(integrator.u, pi.y) annotation(
          Line(points = {{-18, 0}, {-10, 0}}, color = {0, 0, 127}));
        connect(park.y[2], pi.u) annotation(
          Line(points = {{24, 0}, {12, 0}}, color = {0, 0, 127}));
        connect(u, park.u) annotation(
          Line(points = {{108, 0}, {56, 0}}, color = {0, 0, 127}));
        connect(Vqds, park.y) annotation(
          Line(points = {{-104, 50}, {24, 50}, {24, 0}}, color = {0, 0, 127}, thickness = 0.5));
        connect(integrator.y, Phi) annotation(
          Line(points = {{-40, 0}, {-106, 0}}, color = {0, 0, 127}));
        connect(integrator.y, park.theta) annotation(
          Line(points = {{-40, 0}, {-50, 0}, {-50, -38}, {40, -38}, {40, -16}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {2, -3}, lineColor = {0, 0, 255}, extent = {{-52, 35}, {52, -35}}, textString = "PLL"), Text(origin = {-81, -61}, lineColor = {0, 0, 255}, extent = {{-15, 9}, {15, -9}}, textString = "theta")}));
      end PLL;

      model RSC
        import SI = Modelica.Units.SI;
        import pi = Modelica.Constants.pi;
        // Variáveis de referência:
        OmniPES.Units.PerUnit Iqr, Idr;
        // Variáveis dos sensores:
        OmniPES.Units.PerUnit Vqsmed, Vdsmed;
        //
        //  Parâmetros MIT 2MW, 690V, 60Hz (pu):
        parameter OmniPES.Units.PerUnit Ls = 3.1 annotation(
          Dialog(group = "Eletrical Data"));
        parameter OmniPES.Units.PerUnit Lm = 3.0 annotation(
          Dialog(group = "Eletrical Data"));
        parameter Real Htotal = 4.2 + 0.524 annotation(
          Dialog(group = "Mechanical Data"));
        parameter Real Dtotal = 0 annotation(
          Dialog(group = "Mechanical Data"));
        // Parâmetros dos reguladores:
        parameter Real zetaWrm = 1 annotation(
          Dialog(group = "Control Data"));
        parameter Real tsWrm = 5 annotation(
          Dialog(group = "Control Data"));
        // Cálculo dos ganhos:
        parameter Real kiQs = 1 annotation(
          Dialog(group = "Control Data"));
        parameter Real kpWrm = 8 * (2 * Htotal) / tsWrm - Dtotal annotation(
          Dialog(group = "Control Data"));
        parameter Real kiWrm = 16 * (2 * Htotal) / (zetaWrm * tsWrm) ^ 2 annotation(
          Dialog(group = "Control Data"));
        Modelica.Blocks.Continuous.Integrator regQ(initType = Modelica.Blocks.Types.Init.SteadyState, k = kiQs) annotation(
          Placement(visible = true, transformation(origin = {106, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Add add1(k2 = -1) annotation(
          Placement(visible = true, transformation(origin = {76, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Wref annotation(
          Placement(visible = true, transformation(origin = {-100, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput Qsref annotation(
          Placement(visible = true, transformation(origin = {22, 84}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {82, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput Wmed annotation(
          Placement(visible = true, transformation(origin = {-100, 52}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Vs[2] annotation(
          Placement(visible = true, transformation(origin = {-6, 122}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Qsmed annotation(
          Placement(visible = true, transformation(origin = {22, 56}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {111, 71}, extent = {{-11, -11}, {11, 11}}, rotation = 180)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput outMr annotation(
          Placement(visible = true, transformation(origin = {122, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-120, -80}, {-100, -60}}, rotation = 0)));
        Modelica.ComplexBlocks.ComplexMath.RealToComplex MIr annotation(
          Placement(visible = true, transformation(origin = {18, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.WindTurbine.CONTROL_model.PI_ASTROM regW(ki = kiWrm, kp = kpWrm) annotation(
          Placement(visible = true, transformation(origin = {-35, 79}, extent = {{-17, -17}, {17, 17}}, rotation = 0)));
        OmniPES.WindTurbine.CONTROL_model.iPARK ipark annotation(
          Placement(visible = true, transformation(origin = {-34, -14}, extent = {{16, -16}, {-16, 16}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Theta annotation(
          Placement(visible = true, transformation(origin = {-80, -46}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        OmniPES.WindTurbine.CONTROL_model.LIMITER limiter(maxMod = 1.5) annotation(
          Placement(visible = true, transformation(origin = {54, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
// Dados do sensores do estator:
        {Vqsmed, Vdsmed} = Vs;
// Equações de controle:
        regW.y = Vqsmed * (-Lm / Ls * Iqr);
        regQ.y = Vqsmed * ((Vqsmed - Lm * Idr) / Ls);
// Transformação dos sinais de ref:
        {Iqr, Idr} = ipark.u;
        connect(Wref, regW.r) annotation(
          Line(points = {{-100, 80}, {-54, 80}}, color = {0, 0, 127}));
        connect(Wmed, regW.m) annotation(
          Line(points = {{-100, 52}, {-34, 52}, {-34, 60}}, color = {0, 0, 127}));
        connect(Qsref, add1.u1) annotation(
          Line(points = {{22, 84}, {64, 84}}, color = {0, 0, 127}));
        connect(Qsmed, add1.u2) annotation(
          Line(points = {{22, 56}, {52, 56}, {52, 72}, {64, 72}}, color = {0, 0, 127}));
        connect(add1.y, regQ.u) annotation(
          Line(points = {{87, 78}, {93, 78}}, color = {0, 0, 127}));
        connect(Theta, ipark.theta) annotation(
          Line(points = {{-80, -46}, {-34, -46}, {-34, -32}}, color = {0, 0, 127}));
        connect(ipark.y[1], MIr.re) annotation(
          Line(points = {{-16.4, -14}, {-6.4, -14}, {-6.4, -8}, {5.6, -8}}, color = {0, 0, 127}));
        connect(ipark.y[2], MIr.im) annotation(
          Line(points = {{-16.4, -14}, {-6.4, -14}, {-6.4, -20}, {5.6, -20}}, color = {0, 0, 127}));
        connect(limiter.u, MIr.y) annotation(
          Line(points = {{43, -14}, {30, -14}}, color = {85, 170, 255}));
        connect(limiter.y, outMr) annotation(
          Line(points = {{66, -14}, {122, -14}}, color = {85, 170, 255}));
      protected
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {1, 1}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-72, 68}, {72, -68}}, textString = "RSC"), Text(origin = {-31, -154}, lineColor = {255, 255, 255}, extent = {{-11, 6}, {11, -6}}, textString = "Qesp"), Text(origin = {-81, 81}, lineColor = {255, 255, 255}, extent = {{-9, 7}, {9, -7}}, textString = "Wrm"), Text(origin = {82, 81}, lineColor = {255, 255, 255}, extent = {{-10, 7}, {10, -7}}, textString = "Qs"), Text(origin = {-1, -92}, lineColor = {119, 118, 123}, extent = {{-11, 6}, {11, -6}}, textString = "Vccmed", textStyle = {TextStyle.Italic})}),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
      end RSC;

      model GSC
        import SI = Modelica.Units.SI;
        import pi = Modelica.Constants.pi;
        // Variáveis de referência:
        OmniPES.Units.PerUnit Iqg, Idg;
        parameter OmniPES.Units.PerUnit Ceq = 61.5496 annotation(
          Dialog(group = "Converter Data"));
        // Parâmetros dos reguladores:
        parameter Real zetaVcc = 1 annotation(
          Dialog(group = "Control Data"));
        parameter Real tsVcc = 1 annotation(
          Dialog(group = "Control Data"));
        parameter Real kpVcc = 8 * Ceq / tsVcc annotation(
          Dialog(group = "Control Calculed"));
        parameter Real kiVcc = 16 * Ceq / (zetaVcc * tsVcc) ^ 2 annotation(
          Dialog(group = "Control Calculed"));
        Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
          Placement(visible = true, transformation(origin = {-80, 24}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput outMg annotation(
          Placement(visible = true, transformation(origin = {116, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Product product annotation(
          Placement(visible = true, transformation(origin = {-38, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput angulo annotation(
          Placement(visible = true, transformation(origin = {-88, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.ComplexBlocks.ComplexMath.RealToComplex realToComplex annotation(
          Placement(visible = true, transformation(origin = {6, -58}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
        OmniPES.WindTurbine.CONTROL_model.PI_ASTROM Vcc_control(ki = kiVcc, kp = kpVcc) annotation(
          Placement(visible = true, transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.WindTurbine.CONTROL_model.iPARK ipark annotation(
          Placement(visible = true, transformation(origin = {-48, -58}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        LIMITER limiter(maxMod = 1.5) annotation(
          Placement(visible = true, transformation(origin = {44, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      
  Modelica.Blocks.Math.Product product1 annotation(
          Placement(visible = true, transformation(origin = {-38, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step Vccref(height = 0, offset = 1, startTime = 0)  annotation(
          Placement(visible = true, transformation(origin = {-90, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
// Sinais de referência:
        Iqg = Vcc_control.y;
        Idg = 0;
        ipark.u = {Iqg, Idg};
  connect(Vccmed, product.u1) annotation(
          Line(points = {{-80, 24}, {-65, 24}, {-65, 30}, {-50, 30}}, color = {0, 0, 127}));
  connect(product.u2, Vccmed) annotation(
          Line(points = {{-50, 18}, {-65, 18}, {-65, 24}, {-80, 24}}, color = {0, 0, 127}));
  connect(product.y, Vcc_control.m) annotation(
          Line(points = {{-27, 24}, {0, 24}, {0, 49}}, color = {0, 0, 127}));
        connect(angulo, ipark.theta) annotation(
          Line(points = {{-88, -80}, {-48, -80}, {-48, -69}}, color = {0, 0, 127}));
        connect(ipark.y[1], realToComplex.re) annotation(
          Line(points = {{-37, -58}, {-25, -58}, {-25, -52}, {-7, -52}}, color = {0, 0, 127}));
        connect(ipark.y[2], realToComplex.im) annotation(
          Line(points = {{-37, -58}, {-25, -58}, {-25, -64}, {-7, -64}}, color = {0, 0, 127}));
        connect(realToComplex.y, limiter.u) annotation(
          Line(points = {{18, -58}, {34, -58}}, color = {85, 170, 255}));
  connect(limiter.y, outMg) annotation(
          Line(points = {{56, -58}, {116, -58}}, color = {85, 170, 255}));
  connect(product1.y, Vcc_control.r) annotation(
          Line(points = {{-27, 60}, {-12, 60}}, color = {0, 0, 127}));
  connect(Vccref.y, product1.u1) annotation(
          Line(points = {{-63, 60}, {-50, 60}, {-50, 66}}, color = {0, 0, 127}));
  connect(Vccref.y, product1.u2) annotation(
          Line(points = {{-63, 60}, {-50, 60}, {-50, 54}}, color = {0, 0, 127}));
  connect(Vccref.y, product1.u1) annotation(
          Line(points = {{-79, 60}, {-66, 60}, {-66, 66}, {-50, 66}}, color = {0, 0, 127}));
  connect(Vccref.y, product1.u2) annotation(
          Line(points = {{-79, 60}, {-66, 60}, {-66, 54}, {-50, 54}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {3, 1}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-72, 66}, {72, -66}}, textString = "GRC"), Text(origin = {-31, -154}, lineColor = {255, 255, 255}, extent = {{-11, 6}, {11, -6}}, textString = "Qesp"), Text(origin = {-81, 81}, lineColor = {255, 255, 255}, extent = {{-9, 7}, {9, -7}}, textString = "Wrm"), Text(origin = {82, 81}, lineColor = {255, 255, 255}, extent = {{-10, 7}, {10, -7}}, textString = "Qs")}),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
      end GSC;

      model CTRL_MAQ
        import SI = Modelica.Units.SI;
        // CTRL Wrm data:
        parameter SI.Time tsWrm = 3 "Settiling time by speed" annotation(
          Dialog(group = "Velocity Control Data"));
        parameter Units.PerUnit zetaWrm = 1 "Damping constant by speed" annotation(
          Dialog(group = "Velocity Control Data"));
        // CTRL Qs data:
        parameter Units.PerUnit kiQs = 1 "Integral constant by reactivepower" annotation(
          Dialog(group = "Reactive Power Control Data"));
        // CTRL PLL data:
        parameter Units.PerUnit kiPLL = 20 "Integral constant by PLL" annotation(
          Dialog(group = "PLL Control Data"));
        parameter Units.PerUnit kpPLL = 2 "Proporcional constant by PLL" annotation(
          Dialog(group = "PLL Control Data"));
        // CTRL GSC data:
        parameter Units.PerUnit Vccref = 1 "Reference voltage by Vcc" annotation(
          Dialog(group = "GSC Control Data"));
        parameter Units.PerUnit zetaVcc = 1 "Damping constant by Vcc" annotation(
          Dialog(group = "GSC Control Data"));
        parameter SI.Time tsVcc = 3 "Settiling time by Vcc" annotation(
          Dialog(group = "GSC Control Data"));
        // Elec data:
        parameter Units.PerUnit Ls = 3.1 "Stator leakage inductance" annotation(
          Dialog(group = "Electrical Data"));
        parameter Units.PerUnit Lm = 3.0 "Stator magnetizing inductance" annotation(
          Dialog(group = "Electrical Data"));
        // Mech data:
        parameter SI.Time Htotal = 4.2 + 0.524 "Turbine inertia constant" annotation(
          Dialog(group = "Mechanical Data"));
        parameter Units.PerUnit Dtotal = 0.0 "Viscous friction of the turbine" annotation(
          Dialog(group = "Mechanical Data"));
        // Parametros conversor:
        parameter Units.PerUnit Ceq = 35.897 "Capacitor of converter" annotation(
          Dialog(group = "Conversor Data"));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mqdr annotation(
          Placement(visible = true, transformation(origin = {-84, -14}, extent = {{-6, -6}, {6, 6}}, rotation = 0), iconTransformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput Mqdg annotation(
          Placement(visible = true, transformation(origin = {98, -14}, extent = {{6, -6}, {-6, 6}}, rotation = 0), iconTransformation(origin = {50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
          Placement(visible = true, transformation(origin = {58, -42}, extent = {{10, -10}, {-10, 10}}, rotation = -90), iconTransformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.Blocks.Interfaces.RealInput Vqds[2] annotation(
          Placement(visible = true, transformation(origin = {1.42109e-14, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Qmed annotation(
          Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
        Modelica.Blocks.Interfaces.RealInput Wmed annotation(
          Placement(visible = true, transformation(origin = {-87, 17}, extent = {{-9, -9}, {9, 9}}, rotation = 0), iconTransformation(origin = {-110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput Qref annotation(
          Placement(visible = true, transformation(origin = {-30, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealInput Wref annotation(
          Placement(visible = true, transformation(origin = {-64, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {-110, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.WindTurbine.CONTROL_model.RSC rsc(Dtotal = Dtotal, Htotal = Htotal, Lm = Lm, Ls = Ls, kiQs = kiQs, tsWrm = tsWrm, zetaWrm = zetaWrm) annotation(
          Placement(visible = true, transformation(origin = {-47, 1}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
        OmniPES.WindTurbine.CONTROL_model.PLL pll annotation(
          Placement(visible = true, transformation(origin = {0, -48}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.WindTurbine.CONTROL_model.GSC gsc(Ceq = Ceq, tsVcc = tsVcc, zetaVcc = zetaVcc) annotation(
          Placement(visible = true, transformation(origin = {57, 1}, extent = {{-21, -21}, {21, 21}}, rotation = 0)));
      equation
        connect(rsc.Wref, Wref) annotation(
          Line(points = {{-64, 24}, {-64, 42}}, color = {0, 0, 127}));
        connect(rsc.Wmed, Wmed) annotation(
          Line(points = {{-70, 16}, {-86, 16}, {-86, 18}}, color = {0, 0, 127}));
        connect(rsc.Qsref, Qref) annotation(
          Line(points = {{-30, 24}, {-30, 42}}, color = {0, 0, 127}));
        connect(Qmed, rsc.Qsmed) annotation(
          Line(points = {{-4, 16}, {-24, 16}}, color = {0, 0, 127}));
        connect(Vqds, pll.u) annotation(
          Line(points = {{0, -80}, {0, -58}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rsc.Vs, pll.Vqds) annotation(
          Line(points = {{-24, 1}, {6, 1}, {6, -36}}, color = {0, 0, 127}, thickness = 0.5));
        connect(rsc.Theta, pll.Phi) annotation(
          Line(points = {{-24, -14}, {-6, -14}, {-6, -36}}, color = {0, 0, 127}));
        connect(rsc.outMr, Mqdr) annotation(
          Line(points = {{-70, -14}, {-84, -14}}, color = {85, 170, 255}));
        connect(Vccmed, gsc.Vccmed) annotation(
          Line(points = {{58, -42}, {58, -22}}, color = {0, 0, 127}));
        connect(gsc.outMg, Mqdg) annotation(
          Line(points = {{80, -14}, {98, -14}}, color = {85, 170, 255}));
        connect(gsc.angulo, pll.Phi) annotation(
          Line(points = {{34, -14}, {-6, -14}, {-6, -36}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(origin = {0, 40}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 40}, {100, -40}}), Text(origin = {-50, 73}, lineColor = {0, 0, 255}, extent = {{-8, 5}, {8, -5}}, textString = "Ir"), Text(origin = {52, 73}, lineColor = {0, 0, 255}, extent = {{-8, 5}, {8, -5}}, textString = "Ig"), Text(origin = {0, 72}, lineColor = {0, 0, 255}, extent = {{-8, 4}, {8, -4}}, textString = "Vcc"), Text(origin = {91, 11}, lineColor = {0, 0, 255}, extent = {{-9, 5}, {9, -5}}, textString = "Vs"), Text(origin = {86, 42}, lineColor = {0, 0, 255}, extent = {{-10, 8}, {10, -8}}, textString = "Qmed"), Text(origin = {-86, 41}, lineColor = {0, 0, 255}, extent = {{-10, 7}, {10, -7}}, textString = "Wmed"), Text(origin = {-87, 10}, lineColor = {0, 0, 255}, extent = {{-9, 6}, {9, -6}}, textString = "Wref"), Text(origin = {0, 8}, lineColor = {0, 0, 255}, extent = {{-10, 8}, {10, -8}}, textString = "Qref"), Text(origin = {0, 38}, lineColor = {0, 0, 255}, extent = {{-60, 20}, {60, -20}}, textString = "ELEC CTRL")}),
          experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002));
      end CTRL_MAQ;

      model CRTL_TUR
        import pi = Modelica.Constants.pi;
        Modelica.Blocks.Interfaces.RealInput Vw annotation(
          Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-112, 40}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput beta annotation(
          Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealOutput Wrm_opt annotation(
          Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter Real Wrmb = 60 * pi annotation(
          Dialog(group = "Base Data"));
        parameter Real R = 37.5 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real N = 111.5 annotation(
          Dialog(group = "Turbine Data"));
        parameter Real LBD_opt = 6.3279 annotation(
          Dialog(group = "Turbine Data"));
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
        Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, fileName = "/home/uemura/MYCODE/SBSE/Uemura2023SBSE/Modelica/Current Control/mybeta.mat", smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments, tableName = "beta", tableOnFile = true, verboseRead = false) annotation(
          Placement(visible = true, transformation(origin = {8, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      algorithm
// Região 1:
        if Vw >= Vw_min and Vw < Vw_wmin then
          Wrm_opt := N * LBD_opt * Vw_wmin / R / Wrmb;
          beta := 0;
        elseif Vw >= Vw_wmin and Vw < Vw_wmax then
          Wrm_opt := N * LBD_opt * Vw / R / Wrmb;
          beta := 0;
        elseif Vw >= Vw_wmax and Vw < Vw_nom then
          Wrm_opt := N * LBD_opt * Vw_wmax / R / Wrmb;
          beta := 0;
        elseif Vw >= Vw_nom and Vw <= Vw_max then
          Wrm_opt := N * LBD_opt * Vw_wmax / R / Wrmb;
          combiTable1Ds.u := Vw - 12;
          beta := combiTable1Ds.y[1];
        end if;
// Região 2:
// Região 3:
// Região 4:
      equation

        annotation(
          Icon(graphics = {Rectangle(origin = {0, 40}, lineColor = {0, 0, 255}, fillColor = {46, 52, 54}, extent = {{-100, 40}, {100, -40}}), Text(origin = {1, 40}, lineColor = {0, 0, 255}, extent = {{-63, 36}, {63, -36}}, textString = "MECH CTRL")}),
          experiment(StartTime = 0, StopTime = 510, Tolerance = 1e-06, Interval = 0.005));
      end CRTL_TUR;

      model PI_ASTROM
        parameter Real kp = 1, ki = 1;
        Modelica.Blocks.Math.Add DIFERENCE(k2 = -1) annotation(
          Placement(visible = true, transformation(origin = {-28, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Add OUT_ASTROM(k2 = -1) annotation(
          Placement(visible = true, transformation(origin = {34, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Continuous.Integrator INTEGRATOR(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki) annotation(
          Placement(visible = true, transformation(origin = {2, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Math.Gain PROPORTIONAL(k = kp) annotation(
          Placement(visible = true, transformation(origin = {-28, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput r annotation(
          Placement(visible = true, transformation(origin = {-106, 56}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealInput m annotation(
          Placement(visible = true, transformation(origin = {-106, 22}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -112}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
        Modelica.Blocks.Interfaces.RealOutput y annotation(
          Placement(visible = true, transformation(origin = {96, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(INTEGRATOR.y, OUT_ASTROM.u1) annotation(
          Line(points = {{13, 50}, {21, 50}, {21, 34}}, color = {0, 0, 127}));
        connect(PROPORTIONAL.y, OUT_ASTROM.u2) annotation(
          Line(points = {{-17, 22}, {22, 22}}, color = {0, 0, 127}));
        connect(DIFERENCE.y, INTEGRATOR.u) annotation(
          Line(points = {{-17, 50}, {-10, 50}}, color = {0, 0, 127}));
        connect(r, DIFERENCE.u1) annotation(
          Line(points = {{-106, 56}, {-40, 56}}, color = {0, 0, 127}));
        connect(m, PROPORTIONAL.u) annotation(
          Line(points = {{-106, 22}, {-40, 22}}, color = {0, 0, 127}));
        connect(m, DIFERENCE.u2) annotation(
          Line(points = {{-106, 22}, {-60, 22}, {-60, 44}, {-40, 44}}, color = {0, 0, 127}));
        connect(OUT_ASTROM.y, y) annotation(
          Line(points = {{46, 28}, {96, 28}}, color = {0, 0, 127}));
        annotation(
          Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-2, 2}, lineColor = {0, 0, 255}, extent = {{-78, 26}, {78, -26}}, textString = "PI
ASTRÖM"), Text(origin = {-3, 127}, lineColor = {0, 0, 255}, extent = {{-105, 17}, {105, -17}}, textString = "%name")}));
      end PI_ASTROM;

      model LIMITER
        import arg = Modelica.ComplexMath.arg;
        import absC = Modelica.ComplexMath.abs;
        parameter Real maxMod = 1;
        Real absU, faseU, absY, faseY;
        Modelica.ComplexBlocks.Interfaces.ComplexInput u annotation(
          Placement(visible = true, transformation(origin = {-100, -4}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.ComplexBlocks.Interfaces.ComplexOutput y annotation(
          Placement(visible = true, transformation(origin = {86, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        absU = absC(u);
        faseU = arg(u);
        absY = if absU < maxMod then absU else maxMod;
        faseY = faseU;
        absY = absC(y);
        faseY = arg(y);
        annotation(
          Icon(graphics = {Line(points = {{-80, -70}, {-50, -70}, {50, 70}, {80, 70}}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{0, 90}, {-8, 68}, {8, 68}, {0, 90}}), Rectangle(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}}), Line(points = {{-90, 0}, {68, 0}}, color = {192, 192, 192}), Polygon(lineColor = {192, 192, 192}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, points = {{90, 0}, {68, -8}, {68, 8}, {90, 0}}), Line(visible = false, points = {{50, 70}, {80, 70}}, color = {255, 0, 0}), Line(points = {{0, -90}, {0, 68}}, color = {192, 192, 192}), Line(visible = false, points = {{-80, -70}, {-50, -70}}, color = {255, 0, 0})}));
      end LIMITER;
    end CONTROL_model;

    model SENSORS
      extends Modelica.Icons.SensorsPackage;

      model CurrentSensor
        extends Modelica.Icons.RoundSensor;
        OmniPES.Circuit.Interfaces.PositivePin pin_p annotation(
          Placement(visible = true, transformation(origin = {-96, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin pin_n annotation(
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

      model DiferencialVoltageSensor
        extends Modelica.Icons.RoundSensor;
        OmniPES.Circuit.Interfaces.PositivePin pin_p annotation(
          Placement(visible = true, transformation(origin = {-86, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin pin_n annotation(
          Placement(visible = true, transformation(origin = {94, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Interfaces.RealOutput outV[2] annotation(
          Placement(visible = true, transformation(origin = {-44, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      equation
// voltage treatment:
        pin_p.v.re - pin_n.v.re = outV[1];
        pin_p.v.im - pin_n.v.im = outV[2];
// current treatment:
        pin_p.i.re = 0;
        pin_p.i.im = 0;
        pin_n.i.re = 0;
        pin_n.i.im = 0;
        annotation(
          Icon(graphics = {Text(origin = {1, -34}, extent = {{-29, 14}, {29, -14}}, textString = "Voltage")}));
      end DiferencialVoltageSensor;

      model VoltageSensor
        extends Modelica.Icons.RoundSensor;
        OmniPES.Circuit.Interfaces.PositivePin pin_p annotation(
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
        OmniPES.Circuit.Interfaces.PositivePin pin_p annotation(
          Placement(visible = true, transformation(origin = {-96, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.NegativePin pin_n annotation(
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
        S = pin_p.v * conj(pin_p.i);
        outS[1] = S.re;
        outS[2] = S.im;
        annotation(
          Icon(graphics = {Text(origin = {2, -34}, extent = {{-32, 10}, {32, -10}}, textString = "VA")}));
      end PowerSensor;
    equation

    end SENSORS;

    package Examples
      extends Modelica.Icons.ExamplesPackage;

      model testeInfinityBar
        extends Modelica.Icons.Example;
        inner OmniPES.SystemData data(Sbase = 2) annotation(
          Placement(visible = true, transformation(origin = {87, 89}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
        OmniPES.WindTurbine.DFIG_WT dfig_wt(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {-20, 10}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = -6, offset = 12, startTime = 5) annotation(
          Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Step step(height = 0, offset = 0, startTime = 0) annotation(
          Placement(visible = true, transformation(origin = {-70, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Sources.VoltageSource voltageSource annotation(
          Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Interfaces.Bus bus annotation(
          Placement(visible = true, transformation(origin = {20, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter OmniPES.WindTurbine.Interfaces.DWTData smData( Di = 100,Hm(displayUnit = "s"), Ht(displayUnit = "s"), MVAs = data.Sbase, Wb = data.wb, tsVcc = 5, tsWrm = 5, ts_beta(displayUnit = "s")) annotation(
          Placement(visible = true, transformation(origin = {-20, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(step.y, dfig_wt.Qref) annotation(
          Line(points = {{-59, -24}, {-20, -24}, {-20, -8}}, color = {0, 0, 127}));
        connect(dfig_wt.pin_WT, bus.p) annotation(
          Line(points = {{-2, 10}, {20, 10}}, color = {0, 0, 255}));
        connect(voltageSource.p, bus.p) annotation(
          Line(points = {{60, 10}, {20, 10}}, color = {0, 0, 255}));
        connect(ramp.y, dfig_wt.VW) annotation(
          Line(points = {{-59, 10}, {-38, 10}}, color = {0, 0, 127}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.0005));
      end testeInfinityBar;

      model testeRadial
        extends Modelica.Icons.Example;
        inner OmniPES.SystemData data(Sbase = 2) annotation(
          Placement(visible = true, transformation(origin = {87, 89}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
        OmniPES.WindTurbine.DFIG_WT dfig_wt(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {-24, 10}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
        parameter OmniPES.WindTurbine.Interfaces.DWTData smData(Hm(displayUnit = "ks"), Ht(displayUnit = "s"), MVAs = data.Sbase, Wb = data.wb, tsVcc = 1, tsWrm = 5) annotation(
          Placement(visible = true, transformation(origin = {-24, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp ramp(duration = 5, height = 0.75, offset = 11.25, startTime = 5) annotation(
          Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Step step(height = 0, offset = 0, startTime = 0) annotation(
          Placement(visible = true, transformation(origin = {-70, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Sources.VoltageSource voltageSource annotation(
          Placement(visible = true, transformation(origin = {72, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        OmniPES.Circuit.Interfaces.Bus bus annotation(
          Placement(visible = true, transformation(origin = {10, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Circuit.Interfaces.Bus bus1 annotation(
          Placement(visible = true, transformation(origin = {56, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Circuit.Basic.SeriesImpedance seriesImpedance(x = 0.05) annotation(
          Placement(visible = true, transformation(origin = {34, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      equation
        connect(step.y, dfig_wt.Qref) annotation(
          Line(points = {{-59, -24}, {-24, -24}, {-24, -8}}, color = {0, 0, 127}));
        connect(ramp.y, dfig_wt.VW) annotation(
          Line(points = {{-59, 10}, {-43, 10}}, color = {0, 0, 127}));
        connect(dfig_wt.pin_WT, bus.p) annotation(
          Line(points = {{-6.4, 10}, {9.6, 10}}, color = {0, 0, 255}));
        connect(seriesImpedance.n, bus1.p) annotation(
          Line(points = {{44, 9.8}, {56, 9.8}}, color = {0, 0, 255}));
        connect(seriesImpedance.p, bus.p) annotation(
          Line(points = {{24, 10}, {10, 10}}, color = {0, 0, 255}));
        connect(bus1.p, voltageSource.p) annotation(
          Line(points = {{56, 10}, {72, 10}}, color = {0, 0, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 45, Tolerance = 1e-06, Interval = 0.0005));
      end testeRadial;

      model testeInfinityBarFalt
        extends Modelica.Icons.Example;
        inner OmniPES.SystemData data(Sbase = 2) annotation(
          Placement(visible = true, transformation(origin = {87, 89}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
        OmniPES.WindTurbine.DFIG_WT dfig_wt(smData = smData) annotation(
          Placement(visible = true, transformation(origin = {-24, 10}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
        Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = 0, offset = 12, startTime = 0) annotation(
          Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Blocks.Sources.Step step(height = 0, offset = 0.5, startTime = 0) annotation(
          Placement(visible = true, transformation(origin = {-70, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Interfaces.Bus bus annotation(
          Placement(visible = true, transformation(origin = {32, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        parameter Interfaces.DWTData smData( Hm(displayUnit = "s"), Ht(displayUnit = "s"), MVAs = data.Sbase, Wb = data.wb) annotation(
          Placement(visible = true, transformation(origin = {-24, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        OmniPES.Circuit.Sources.ControlledVoltageSource controlledVoltageSource annotation(
          Placement(visible = true, transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
        Modelica.ComplexBlocks.Sources.ComplexStep complexStep(height = Complex(-0.8, 0), offset = Complex(1, 0), startTime = 1) annotation(
          Placement(visible = true, transformation(origin = {126, 6}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.ComplexBlocks.Sources.ComplexStep complexStep1(height = Complex(0.8, 0), offset = Complex(0), startTime = 1.1) annotation(
          Placement(visible = true, transformation(origin = {126, -34}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
        Modelica.ComplexBlocks.ComplexMath.Add add annotation(
          Placement(visible = true, transformation(origin = {86, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      equation
        connect(step.y, dfig_wt.Qref) annotation(
          Line(points = {{-59, -24}, {-24, -24}, {-24, -8}}, color = {0, 0, 127}));
        connect(ramp.y, dfig_wt.VW) annotation(
          Line(points = {{-59, 10}, {-43, 10}}, color = {0, 0, 127}));
        connect(dfig_wt.pin_WT, bus.p) annotation(
          Line(points = {{-6.4, 10}, {32, 10}}, color = {0, 0, 255}));
        connect(controlledVoltageSource.p, bus.p) annotation(
          Line(points = {{60, 10}, {32, 10}}, color = {0, 0, 255}));
        connect(controlledVoltageSource.u, add.y) annotation(
          Line(points = {{68, 0}, {76, 0}}, color = {85, 170, 255}));
        connect(add.u1, complexStep.y) annotation(
          Line(points = {{98, 6}, {116, 6}}, color = {85, 170, 255}));
        connect(add.u2, complexStep1.y) annotation(
          Line(points = {{98, -6}, {106, -6}, {106, -34}, {116, -34}}, color = {85, 170, 255}));
      protected
        annotation(
          experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.0005));
      end testeInfinityBarFalt;
    end Examples;
  end WindTurbine;
  annotation(
    uses(Modelica(version = "4.0.0")));
end OmniPES;