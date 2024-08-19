//
//  BTUUIDs.swift
//  BLEDemo
//
//  Created by Jindrich Dolezy on 28/11/2018.
//  Copyright © 2018 Dzindra. All rights reserved.
//

import CoreBluetooth


struct BTUUIDs {
    static let blinkUUID = CBUUID(string: "e94f85c8-7f57-4dbd-b8d3-2b56e107ed60")
    
    static let esp32Service = CBUUID(string: "9a8ca9ef-e43f-4157-9fee-c37a3d7dc12d")
    
    static let servoUUID = CBUUID(string: "f74fb3de-61d1-4f49-bd77-419b61d188da")
    
    static let bno08xUUID = CBUUID(string: "c91b34b8-90f3-4fee-89f7-58c108ab198f")
    
    static let pidUUID = CBUUID(string: "a979c0ba-a2be-45e5-9d7b-079b06e06096")
    
    static let utilitiesUUID = CBUUID(string: "fb02a2fa-2a86-4e95-8110-9ded202af76b")
    
    //    #define UUID_7 "83e6a4bd-8347-409c-87f3-d8c896f15d3d"
    //    #define UUID_8 "680f38b9-6898-40ea-9098-47e30e97dbb5"
}
