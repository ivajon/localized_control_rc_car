//! Defines an asynchronously populated graph that get populated using
//! [`channels`](tokio::sync::mpsc).

use std::{collections::HashMap, sync::Arc};

use ratatui::{
    layout::Alignment,
    style::{Style, Stylize},
    symbols::Marker,
    widgets::{block::Title, Axis, Block, Borders, Chart, Dataset, Widget},
};
use tokio::sync::{mpsc, Mutex};

pub type GraphValue = (Vec<(f64, f64)>, Style, f64, f64);

#[derive(Clone)]
/// A generic plot that allows the user to asynchronously add points to the plot.
pub struct Graph {
    values: HashMap<String, GraphValue>,
    latest: f64,
    /// Time to leak some memory boys.
    widget: Chart<'static>,
    redraw_writer: RedrawWriter,
}

type MeasurementReader = mpsc::Receiver<(String, (f64, f64))>;
/// Write to this channel to add a new value to the graph.
pub type MeasurementWriter = mpsc::Sender<(String, (f64, f64))>;
/// A composite return type that contains all of the join handles and the
/// channel that allows the user to add points to the graph.
pub type GraphConstructor = (
    Arc<Mutex<Box<Graph>>>,
    MeasurementWriter,
    Vec<tokio::task::JoinHandle<()>>,
);

// Internals
/// Writing to this channel will re-draw the ui.
type RedrawWriter = mpsc::Sender<()>;

impl Graph {
    /// Initiates a new graph element and returns all of the join handles
    /// that are needed to kill the process if needed.
    pub fn init(redraw_writer: RedrawWriter) -> GraphConstructor {
        let ret = Arc::new(Mutex::new(Box::new(Self {
            values: HashMap::new(),
            latest: 0.,
            widget: Chart::new(Vec::new()),
            redraw_writer,
        })));

        let (sender_external, rec_external) = mpsc::channel(1024);
        let (sender_internal, rec_internal) = mpsc::channel(1024);

        (
            ret.clone(),
            sender_external,
            // Spawn all of the needed tasks.
            vec![
                tokio::spawn(Self::plot(ret.clone(), rec_internal)),
                tokio::spawn(Self::measurement_register(
                    ret.clone(),
                    rec_external,
                    sender_internal,
                )),
            ],
        )
    }

    /// Returns a drawable widget with the latest information.
    pub fn widget(&self) -> impl Widget {
        self.widget.clone()
    }

    /// Renders a new [`Widget`] that contains all of the latest data, this method
    /// will only run whenever there is any data to render.
    async fn plot(graph: Arc<Mutex<Box<Self>>>, mut reader: mpsc::Receiver<()>) {
        // Only re-draw the ui when ever a new measurement is registered.
        while reader.recv().await.is_some() {
            {
                let graph_inner = &mut graph.lock().await;

                let mut datasets = Vec::new();
                let mut max = -f64::INFINITY;
                let mut min = f64::INFINITY;

                // This really has to be re-written if we are going to use this in anything real.
                graph_inner.values.clone().into_iter().for_each(
                    |(id, (data, style, inner_max, inner_min))| {
                        if max < inner_max {
                            max = inner_max;
                        }
                        if min > inner_min {
                            min = inner_min;
                        }
                        datasets.push(
                            Dataset::default()
                                .name(id.clone())
                                // This scares me.
                                // Not as much when we re-draw only on measurements
                                .data(data.leak())
                                .style(style)
                                .marker(Marker::Dot)
                                .clone(),
                        )
                    },
                );

                // This is probably horrendously slow.
                let new_chart = Chart::new(datasets.clone())
                    .block(
                        Block::default()
                            .title(
                                Title::default()
                                    .content("Line chart".cyan().bold())
                                    .alignment(Alignment::Center),
                            )
                            .borders(Borders::ALL),
                    )
                    .x_axis(
                        Axis::default()
                            .bounds([graph_inner.latest - 1000., graph_inner.latest])
                            .title("Time [uS]"),
                    )
                    .y_axis(
                        Axis::default()
                            .bounds([min, max])
                            .labels(vec![
                                format!("min : {min:.1}").into(),
                                format!("max : {max:.1}").into(),
                            ])
                            .title("Velocity [cm/s]"),
                    );
                graph_inner.widget = new_chart;
                let _ = graph_inner.redraw_writer.send(()).await;
            }
        }
    }

    /// Registers a new measurement in the graph, if it belongs to a dataset that has not been
    /// introduced yet it will grab the next color in line, otherwise it will add it to the
    /// previous dataset.
    async fn measurement_register(
        graph: Arc<Mutex<Box<Self>>>,
        mut reader: MeasurementReader,
        sender: mpsc::Sender<()>,
    ) {
        let styles = [
            Style::default().green(),
            Style::default().yellow(),
            Style::default().blue(),
            Style::default().white(),
            Style::default().cyan(),
        ];
        let mut ptr = 0;

        while let Some((name, (x, y))) = reader.recv().await {
            let mut graph_inner = graph.lock().await;
            graph_inner.latest = x;
            match graph_inner.values.get_mut(&name) {
                Some(data) => {
                    if data.2 < y {
                        data.2 = y;
                    }
                    if data.3 > y {
                        data.3 = y;
                    }
                    data.0.push((x, y));
                }
                None => {
                    let style = match styles.get(ptr) {
                        Some(style) => *style,
                        None => {
                            ptr = 0;
                            styles[0]
                        }
                    };
                    ptr += 1;
                    graph_inner.values.insert(name, (vec![(x, y)], style, y, y));
                }
            }

            match sender.send(()).await {
                Ok(_) => {}
                _ => break,
            }
        }
    }
}
