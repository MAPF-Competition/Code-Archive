class Instance {
    constructor(map_name, input) {
        this.set_map(map_name)
        this.set_solution(input)
    }

    set_map(map_name) {
        this.map = data['instances'][map_name]
        // delete this.map.bitmap
    }

    set_solution(input) {
        this.solution = {}
        if (input !== "") {
            this.solution = JSON.parse(input)
        }
    }

    location_to_coords(location) {
        const x = location % this.map['width']
        const y = Math.floor(location / this.map['width'])
        return [x, y]
    }

    coords_to_location(x, y) {
        return y*this.map['width'] + x
    }

    obstacle(location) {
        const c = this.map['bitmap'].charAt(location)
        return c === '@' || c === 'T'
    }

    can_move(location, direction) {
        // return true
        const coords = this.location_to_coords(location)
        if (direction === 0) {
            return (coords[0]+1 < this.map['width'] && !this.obstacle(location + 1))
        } else if (direction == 1) {
            return (coords[1]+1 < this.map['height'] && !this.obstacle(location + this.map['width']))
        } else if (direction === 2) {
            return (coords[0]-1 >= 0 && !this.obstacle(location - 1))
        } else {
            return (coords[1]-1 >= 0 && !this.obstacle(location - this.map['width']))
        }
    }
}


class Application {
    constructor() {
        const height = 0.8 * window.innerHeight
        this.svg = SVG().addTo('#app').size(height, height)
        this.svg.group().attr({id: "pixels"})
        this.svg.group().attr({id: "arrows"})
        this.svg.panZoom({ zoomFactor: 1.2 })
        this.instance = []
        this.set_instance('random-32-32-20')
        this.update_solution()
    }

    draw() {
        const pixels = this.svg.findOne('g#pixels')
        pixels.clear()
        pixels.path(this.instance.map['path']).fill('black')
        let pattern = this.svg.pattern(10, 10, function(add) {
            add.circle(2).move(4, 4)
        })
        pixels.rect(10*this.instance.map['width'], 10*this.instance.map['height']).attr("fill", pattern)
        const arrows = this.svg.findOne('g#arrows')
        arrows.clear()
        let i = 0
        for (let y = 0; y < this.instance.map['height']; y++) {
            for (let x = 0; x < this.instance.map['width']; x++) {
                if (!this.instance.obstacle(i)) {
                    if (!this.instance.solution.E.includes(i) && this.instance.can_move(i, 0)) {
                        arrows.polyline('0,0 1.5,1.5 0,3').fill('none').stroke({ color: '#f06', width: 1 }).move(10*x+8.5, 10*y+3.5)
                    }
                    if (!this.instance.solution.S.includes(i) && this.instance.can_move(i, 1)) {
                        arrows.polyline('0,0 1.5,1.5 3,0').fill('none').stroke({ color: '#f06', width: 1 }).move(10*x+3.5, 10*y+8.5)
                    }
                    if (!this.instance.solution.W.includes(i) && this.instance.can_move(i, 2)) {
                        arrows.polyline('0,0 -1.5,1.5 0,3').fill('none').stroke({ color: '#f06', width: 1 }).move(10*x+0, 10*y+3.5)
                    }
                    if (!this.instance.solution.N.includes(i) && this.instance.can_move(i, 3)) {
                        arrows.polyline('0,0 1.5,-1.5 3,0').fill('none').stroke({ color: '#f06', width: 1 }).move(10*x+3.5, 10*y+0)
                    }
                }
                i++
            }
        }
        this.reset_viewer()
    }

    set_instance(map_name) {
        const input = document.querySelector('textarea').value
        this.instance = new Instance(map_name, input)
        // this.update_solution()
        this.draw()
        // this.set_time(0)

        const svg = document.querySelector('svg')
        svg.addEventListener('mousemove', (e) => {
            let point = svg.createSVGPoint()
            point.x = e.clientX
            point.y = e.clientY
            point = point.matrixTransform(svg.getScreenCTM().inverse())
            const x = Math.floor(point.x / 10)
            const y = Math.floor(point.y / 10)
            const location = this.instance.coords_to_location(x, y)
            document.querySelector('#coords').textContent = `(${x}, ${y}; ${location})`
        })
    }

    update_solution() {
        document.querySelector("button#edit").innerHTML = 'Load solution <i class="bi bi-check"></i>'
        const input = document.querySelector('textarea').value
        this.instance.set_solution(input)
        this.draw()
        document.querySelector("button#edit").innerHTML = 'Load solution <i class="bi bi-check-all"></i>'
    }

    reset_viewer() {
        const w = 10*this.instance.map['width']
        const h = 10*this.instance.map['height']
        this.svg.viewbox(`0 0 ${w} ${h}`)
    }

    show_coordinates(location) {
        let coords = this.instance.location_to_coords(location)
        document.querySelector("#location-coords").textContent = `(x:${coords[0]}, y:${coords[1]})`
    }

}