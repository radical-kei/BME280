//
//  TempHumApp.swift
//  TempHumApp
//
//  Created by radical on 2021/04/04.
//

import SwiftUI

@main
struct TempHumApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(TempHum(host: "192.168.1.43",port: "50000"))
        }
    }
}
