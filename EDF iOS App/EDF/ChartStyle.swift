import Foundation
import SwiftUICharts
import SwiftUI

struct ChartStyle {
    let gridStyle = GridStyle(lineColour   : Color(.lightGray).opacity(0.5),
                              lineWidth    : 1,
                              dash         : [8],
                              dashPhase    : 0)
    
    var chartStyle : LineChartStyle
    
    init() {
        chartStyle = LineChartStyle(infoBoxPlacement    : .floating,
                                    infoBoxBorderColour : Color.primary,
                                    infoBoxBorderStyle  : StrokeStyle(lineWidth: 1),
                                    
                                    markerType          : .vertical(attachment: .line(dot: .style(DotStyle()))),
                                    
                                    xAxisGridStyle      : gridStyle,
                                    xAxisLabelPosition  : .bottom,
                                    xAxisLabelColour    : Color.primary,
                                    xAxisLabelsFrom     : .dataPoint(rotation: .degrees(0)),
                                    
                                    yAxisGridStyle      : gridStyle,
                                    yAxisLabelPosition  : .leading,
                                    yAxisLabelColour    : Color.primary,
                                    
                                    globalAnimation     : .easeOut(duration: 0))
    }
    
    func getChartStyle() -> LineChartStyle {
        return chartStyle
    }
    
    @ViewBuilder func getGraph(datasets: LineDataSet, colour: Color) -> some View {
        let chartData = LineChartData(dataSets: datasets, chartStyle: ChartStyle().getChartStyle())
        LineChart(chartData: chartData)
            .filledTopLine(chartData: chartData,
                           lineColour: ColourStyle(colour: colour),
                           strokeStyle: StrokeStyle(lineWidth: 3))
            .touchOverlay(chartData: chartData, specifier: "%.2f")
            .xAxisGrid(chartData: chartData)
            .yAxisGrid(chartData: chartData)
            .xAxisLabels(chartData: chartData)
            .yAxisLabels(chartData: chartData, specifier: "%.2f")
            .floatingInfoBox(chartData: chartData)
            .id(chartData.id)
            .frame(minWidth: 150, maxWidth: 390, minHeight: 150, maxHeight: 400)
    }
}
