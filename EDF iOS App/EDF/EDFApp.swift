//
//  ESP_32_InterfaceApp.swift
//  ESP 32 Interface
//
//  Created by Eugene on 7/23/24.
//

import SwiftUI

@main
struct EDFApp: App {
    @StateObject var bluetoothDevice = BluetoothDeviceHelper()
    @StateObject var edf = EDF()
    
    var body: some Scene {
        WindowGroup {
            HomeScreenView()
                .environmentObject(bluetoothDevice)
                .environmentObject(edf)
        }
    }
}

