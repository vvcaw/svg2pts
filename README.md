# svg2pts

[![](https://meritbadge.herokuapp.com/svg2pts)](https://crates.io/crates/svg2pts)
<!-- [![](https://docs.rs/svg2pts/badge.svg)](https://docs.rs/svg2pts/) -->

Command line utility to convert the paths in a SVG to a list of points. All paths in the SVG is converted into a list of points
curve interpolation is controlled by command line arguments. 


* [Installation](#installation)
* [Usage](#usage)
* [Examples](#usage)

<a name="Installation"></a>

## Installation

Using cargo: 
```sh
cargo install svg2pts
```

This will make the svg2pts binary available in your cargo binary directory; usually `~/.cargo/bin`.

## Usage

```text
svg2pts 0.1.2
Converts all paths in a svg to a list of points.

USAGE:
    svg2pts [OPTIONS] [ <input> [<output>] ]

FLAGS:
    -h, --help       Prints help information

OPTIONS:
    -a, --accuracy <accuracy>    Set target accuracy for bezier curve [default: 0.1]
    -d, --distance <distance>    Set target distance between points, depends on DPI of SVG.
                                 If distance == 0.0 point distance not normalized.
                                 [default: 0.0]

ARGS:
    <input>     Input SVG file, stdin if not present
    <output>    Output file, stdout if not present"#
```

## Examples

<p align="center">

```sh
$ svg2pts -d 3.5 media/rust.svg
71.05 120.32
67.5572042727889 120.0955495422685
64.09895790373652 119.5565392910338
60.64071153468416 119.0175290397991
57.283778022261004 118.02707591462337
...

$ svg2pts -d 3.5 media/rust.svg | gnuplot -p -e 'plot "<cat"'
#graphs below
```

### SVG converted to points with distance 1.5

<img
width="368"
src="https://raw.githubusercontent.com/exrok/svg2pts/master/media/plot1.png"
/>

### SVG converted to points with distance 3.5

<img
width="368"
src="https://raw.githubusercontent.com/exrok/svg2pts/master/media/plot2.png"
/>

### SVG converted to points without normalized distance

<img
width="368"
src="https://raw.githubusercontent.com/exrok/svg2pts/master/media/plot3.png"
/>

### SVG converted without normalized distance and lower accuracy (simplified)

<img
width="490"
src="https://raw.githubusercontent.com/exrok/svg2pts/master/media/plot4.png"
/>

### SVG converted to points then displayed on an oscilloscope

```sh
svg2pts -d 3.5 rust.svg | pts2wav > logo.wav
```

<img
  src="https://raw.githubusercontent.com/exrok/svg2pts/master/media/rustlogo_osc.gif"
  alt="Rust logo svg converted to pts and display on oscilloscope."
  width=256
/>

You can get `pts2wav` here [https://github.com/exrok/pts2wav](https://github.com/exrok/pts2wav)
</p>

