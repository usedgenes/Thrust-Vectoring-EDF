//
//  BluetoothDeviceHelper.swift
//  ESP 32 Interface
//
//  Created by Eugene on 7/26/24.
//

import Foundation
import UserNotifications


class BluetoothDeviceHelper: ObservableObject {
    @Published var refreshBluetooth: Bool = false
    
    @Published var deviceName: String = "null"
    @Published var isConnected: Bool = false
        
    @Published var device: BTDevice? {
        didSet {
            device?.delegate = self
        }
    }
    
    @Published var edf: EDF? {
        didSet {
            device?.edf = edf
        }
    }
    
    init() {
        device?.delegate = self
    }
    
    func refresh() {
        refreshBluetooth.toggle()
    }
    
    func disconnect() {
        device?.disconnect()
        deviceName = "null"
        isConnected = false
    }
    
    func connect() {
        if(device != nil) {
            isConnected = true
            deviceName = device?.name ?? "null"
        }
    }
    
    func blinkChanged() {
        device?.blink.toggle()
    }
    
    func blinkState() -> Bool {
        if(device?.blink == nil) {
            return false
        }
        return device!._blink
    }
    
    func setServos(input : String) {
        device!.servoString = input
    }
    
    func setBNO08X(input: String) {
        device!.bno08xString = input
    }
    
    func setPID(input: String) {
        device!.pidString = input
    }
    
    func setUtilities(input: String) {
        device!.utilitiesString = input
    }
}

extension BluetoothDeviceHelper: BTDeviceDelegate {
    func deviceSerialChanged(value: String) {
        
    }
    
    func deviceSpeedChanged(value: Int) {
    }
    
    func deviceConnected() {
    }
    
    func deviceDisconnected() {
    }
    
    func deviceReady() {
    }
    
    func deviceBlinkChanged(value: Bool) {
        
    }
}
    
