//
//  NearbyDevicesVC.swift
//  BLEDemo
//
//  Created by Jindrich Dolezy on 28/11/2018.
//  Copyright © 2018 Dzindra. All rights reserved.
//
import CoreBluetooth


class BluetoothManagerHelper : ObservableObject {
    
    @Published var manager = BTManager()
    @Published var device: BTDevice?
    
    @Published var devices: [BTDevice] = [] {
        didSet {
        }
    }
    
    init() {
        manager.delegate = self
    }
    
    func connectDevice(BTDevice : BTDevice) {
        device = BTDevice
        device?.connect()
    }
}

extension BluetoothManagerHelper: BTManagerDelegate {
    func didChangeState(state: CBManagerState) {
        devices = manager.devices
    }
    
    func didDiscover(device: BTDevice) {
        devices = manager.devices
    }
    
    func didEnableScan(on: Bool) {
        
    }
    
    
}
