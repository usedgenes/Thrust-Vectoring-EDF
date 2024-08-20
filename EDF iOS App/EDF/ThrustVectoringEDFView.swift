import SwiftUI
import SwiftUICharts

struct ThrustVectoringEDFView: View {
    @EnvironmentObject var bluetoothDevice : BluetoothDeviceHelper
    @EnvironmentObject var edf : EDF
    
    @State var servo0Position : Double = 0
    @State var servo1Position : Double = 0
    @State var servo2Position : Double = 0
    @State var servo3Position : Double = 0
    @State var edfPower : Double = 50
    
    @State var rollKp : String = "50.0"
    @State var rollKi : String = "0.5"
    @State var rollKd : String = "0.0"
    @State var pitchKp : String = "50.0"
    @State var pitchKi : String = "0.0"
    @State var pitchKd : String = "0.0"
    @State var yawKp : String = "50.0"
    @State var yawKi : String = "0.0"
    @State var yawKd : String = "0.0"
    
    var body: some View {
        ScrollView {
            Section {
                HStack {
                    Text("Loop Time: " + String(edf.loopTime) + " ms")
                }
                Text("PID Values")
                    .frame(maxWidth: .infinity, alignment: .center)
                HStack {
                    Button(action: {
                        bluetoothDevice.setPID(input: "0" + rollKp + "," + rollKi + "!" + rollKd)
                        bluetoothDevice.setPID(input: "1" + pitchKp + "," + pitchKi + "!" + pitchKd)
                        bluetoothDevice.setPID(input: "2" + yawKp + "," + yawKi + "!" + yawKd)
                    }) {
                        Text("Apply")
                    }.buttonStyle(BorderlessButtonStyle())
                        .frame(maxWidth: .infinity, alignment: .center)
                    Button(action: {
                        bluetoothDevice.setUtilities(input: "0")
                    }) {
                        Text("RESET ESP32")
                    }.buttonStyle(BorderlessButtonStyle())
                        .frame(maxWidth: .infinity, alignment: .center)
                }.padding(.bottom)
                HStack {
                    Text("Roll:")
                    Text("Kp:")
                    TextField(rollKp, text: Binding<String>(
                        get: { rollKp },
                        set: {
                            rollKp = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                    Text("Ki:")
                    TextField(rollKi, text: Binding<String>(
                        get: { rollKi },
                        set: {
                            rollKi = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                    Text("Kd:")
                    TextField(rollKd, text: Binding<String>(
                        get: { rollKd },
                        set: {
                            rollKd = $0
                            
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                }
                HStack {
                    Text("Pitch:")
                    Text("Kp:")
                    TextField(pitchKp, text: Binding<String>(
                        get: { pitchKp },
                        set: {
                            pitchKp = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                    Text("Ki:")
                    TextField(pitchKi, text: Binding<String>(
                        get: { pitchKi },
                        set: {
                            
                            pitchKi = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                    Text("Kd:")
                    TextField(pitchKd, text: Binding<String>(
                        get: { pitchKd },
                        set: {
                            pitchKd = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                }
                HStack {
                    Text("Yaw:")
                    Text("Kp:")
                    TextField(yawKp, text: Binding<String>(
                        get: { yawKp },
                        set: {
                            yawKp = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                    Text("Ki:")
                    TextField(yawKi, text: Binding<String>(
                        get: { yawKi },
                        set: {
                            yawKi = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                    Text("Kd:")
                    TextField(yawKd, text: Binding<String>(
                        get: { yawKd },
                        set: {
                            yawKd = $0
                        }))
                    .keyboardType(UIKeyboardType.decimalPad)
                }
            }.padding(.leading)
            
            Section {
                HStack {
                    Text("Servo 0: " + String(Int(servo0Position)))
                        .padding(.trailing)
                    Slider(value: Binding(get: {
                        servo0Position
                    }, set: { (newVal) in
                        servo0Position = newVal
                    }), in: -30...30, step: 1) { editing in
                        if(!editing) {
                            bluetoothDevice.setServos(input: "0" + String(Int(servo0Position)))
                        }
                    }
                }.padding()
                HStack {
                    Text("Servo 1: " + String(Int(servo1Position)))
                        .padding(.trailing)
                    Slider(value: Binding(get: {
                        servo1Position
                    }, set: { (newVal) in
                        servo1Position = newVal
                    }), in: -30...30, step: 1) { editing in
                        if(!editing) {
                            bluetoothDevice.setServos(input: "1" + String(Int(servo1Position)))
                        }
                    }
                }.padding()
                HStack {
                    Text("Servo 2: " + String(Int(servo2Position)))
                        .padding(.trailing)
                    Slider(value: Binding(get: {
                        servo2Position
                    }, set: { (newVal) in
                        servo2Position = newVal
                    }), in: -30...30, step: 1) { editing in
                        if(!editing) {
                            bluetoothDevice.setServos(input: "2" + String(Int(servo2Position)))
                        }
                    }
                }.padding()
                HStack {
                    Text("Servo 3: " + String(Int(servo3Position)))
                        .padding(.trailing)
                    Slider(value: Binding(get: {
                        servo3Position
                    }, set: { (newVal) in
                        servo3Position = newVal
                    }), in: -30...30, step: 1) { editing in
                        if (!editing) {
                            bluetoothDevice.setServos(input: "3" + String(Int(servo3Position)))
                        }
                    }
                }.padding()
            }
            HStack {
                Text("EDF Power: " + String(Int(edfPower)))
                    .padding(.trailing)
                Slider(value: Binding(get: {
                    edfPower
                }, set: { (newVal) in
                    edfPower = newVal
                }), in: 50...180, step: 1) { editing in
                    if(!editing) {
                        bluetoothDevice.setServos(input: "4" + String(Int(edfPower)))
                    }
                }
            }.padding()
            GroupBox {
                NavigationLink("View Orientation:", destination: edfGraphView())
                    .padding()
            }
            GroupBox {
                NavigationLink("View Servo Values:", destination: edfServoPosView())
                    .padding()
            }
            GroupBox {
                NavigationLink("View PID Values:", destination: edfPidView())
                    .padding()
            }
        }.hideKeyboardWhenTappedAround()
            .navigationViewStyle(StackNavigationViewStyle())
    }
}

struct edfGraphView : View {
    @EnvironmentObject var edf : EDF
    @EnvironmentObject var bluetoothDevice : BluetoothDeviceHelper
    @State var getData = false
    
    var body : some View {
        Section {
            Text("BMI088 Data Graphs")
                .frame(maxWidth: .infinity, alignment: .center)
                .padding()
            HStack {
                Button(action: {
                    bluetoothDevice.setBNO08X(input: "11")
                    getData.toggle()
                }) {
                    Text("Get Data")
                }.disabled(getData)
                    .buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
                
                Button(action: {
                    bluetoothDevice.setBNO08X(input: "10")
                    getData.toggle()
                }) {
                    Text("Stop")
                }.disabled(!getData)
                    .buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
                Button(action: {
                    edf.resetRotation()
                }) {
                    Text("Reset All")
                }.buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
                Button(action: {
                    bluetoothDevice.setBNO08X(input: "1")
                }) {
                    Text("Calibrate")
                }.buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
            }.padding(.bottom)
        }.onDisappear(perform: {
            bluetoothDevice.setBNO08X(input: "10")
        })
        
        Text("Yaw")
        ChartStyle().getGraph(datasets: edf.getYaw(), colour: .red)
        
        Text("Yaw Velocity")
        ChartStyle().getGraph(datasets: edf.getYawVelocity(), colour: .red)
        
        Text("Pitch")
        ChartStyle().getGraph(datasets: edf.getPitch(), colour: .green)
        
        Text("Roll")
        ChartStyle().getGraph(datasets: edf.getRoll(), colour: .blue)
    }
}

struct edfServoPosView : View {
    @EnvironmentObject var edf : EDF
    @EnvironmentObject var bluetoothDevice : BluetoothDeviceHelper
    @State var getData = false
    
    var body: some View {
        Section {
            Text("Servo Position Graphs")
                .frame(maxWidth: .infinity, alignment: .center)
                .padding()
            HStack {
                Button(action: {
                    bluetoothDevice.setServos(input: "11")
                    getData.toggle()
                }) {
                    Text("Get Data")
                }.disabled(getData)
                    .buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
                
                Button(action: {
                    bluetoothDevice.setServos(input: "10")
                    getData.toggle()
                }) {
                    Text("Stop")
                }.disabled(!getData)
                    .buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
                Button(action: {
                    edf.resetServoPos()
                }) {
                    Text("Reset All")
                }.buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
            }.padding(.bottom)
        }.onDisappear(perform: {
            bluetoothDevice.setServos(input: "10")
        })
        
        Text("Servo 0 Position")
        ChartStyle().getGraph(datasets: edf.getServo0Pos(), colour: .red)
        
        Text("Servo 1 Position")
        ChartStyle().getGraph(datasets: edf.getServo1Pos(), colour: .green)
        
        Text("Servo 2 Position")
        ChartStyle().getGraph(datasets: edf.getServo2Pos(), colour: .blue)
        
        Text("Servo 3 Position")
        ChartStyle().getGraph(datasets: edf.getServo3Pos(), colour: .yellow)
    }
}

struct edfPidView : View {
    @EnvironmentObject var edf : EDF
    @EnvironmentObject var bluetoothDevice : BluetoothDeviceHelper
    @State var getData = false
    
    var body: some View {
        Section {
            Text("PID Command Graphs")
                .frame(maxWidth: .infinity, alignment: .center)
                .padding()
            HStack {
                Button(action: {
                    bluetoothDevice.setPID(input: "31")
                    getData.toggle()
                }) {
                    Text("Get Data")
                }.disabled(getData)
                    .buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
                
                Button(action: {
                    bluetoothDevice.setPID(input: "30")
                    getData.toggle()
                }) {
                    Text("Stop")
                }.disabled(!getData)
                    .buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
                Button(action: {
                    edf.resetPIDCommands()
                }) {
                    Text("Reset All")
                }.buttonStyle(BorderlessButtonStyle())
                    .frame(maxWidth: .infinity, alignment: .center)
            }.padding(.bottom)
        }.onDisappear(perform: {
            bluetoothDevice.setPID(input: "30")
        })
        
        Text("Yaw Command")
        ChartStyle().getGraph(datasets: edf.getYawCommand(), colour: .red)
        
        Text("Pitch Command")
        ChartStyle().getGraph(datasets: edf.getPitchCommand(), colour: .green)
        
        Text("Roll Command")
        ChartStyle().getGraph(datasets: edf.getRollCommand(), colour: .blue)
    }
}

struct ThrustVectoringEDFView_Previews: PreviewProvider {
    static var previews: some View {
        ThrustVectoringEDFView()
    }
}

extension View {
    func hideKeyboardWhenTappedAround() -> some View  {
        return self.onTapGesture {
            UIApplication.shared.sendAction(#selector(UIResponder.resignFirstResponder),
                                            to: nil, from: nil, for: nil)
        }
    }
}
