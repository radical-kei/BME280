//
//  ContentView.swift
//  TempHumApp
//
//  Created by radical on 2021/04/04.
//

import SwiftUI

struct ContentView: View{
    @EnvironmentObject var temphum : TempHum
    
    var body: some View {
        VStack{
            Text("マイルーム")
                .font(.title)
            Text(temphum.data.temp)
            Text(temphum.data.hum)
            Text(temphum.data.press)
        }
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        Group {
            ContentView()
                .environmentObject(TempHum(host: "192.168.1.43",port: "50000"))
        }
    }
}
