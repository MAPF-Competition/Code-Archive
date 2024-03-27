/* Given a map and a solution:
   - Load both
   - Define the position (location and direction) of every agent at every time
   - Interpret the messages (task done, task assigned)
*/
class Instance {
    constructor(map_name, input) {
        this.set_map(map_name)
        this.set_solution(input)
    }

    set_map(map_name) {
        this.map = data['instances'][map_name]
        delete this.map.bitmap
    }

    set_solution(input) {
        this.solution = {}
        if (input !== "") {
            this.solution = JSON.parse(input)
            this.set_paths()
            this.set_moves()
            this.count_finished()
        }
    }

    // Set the timesteps where there is a move, linecap: 'round'
    set_moves() {
        this.wait = []
        for (let t = 0; t < this.solution.makespan; t++) {
            this.wait.push(this.solution.plannerPaths[0].charAt(2*t) === 'T')
        }
    }

    set_paths() {
        this.agents = []
        for (let a = 0; a < this.solution.teamSize; a++) {
            const dir = {"E": 0, "S": 1, "W": 2, "N": 3}
            let path = [{x: this.solution.start[a][1], y: this.solution.start[a][0], direction: dir[this.solution.start[a][2]]}]
            for (let t = 0; t < this.solution.makespan; t++) {
                let step = JSON.parse(JSON.stringify(path[t]))
                let move = this.solution.plannerPaths[a].charAt(2*t)
                if (move === 'F') {
                    if (step.direction === 0) {
                        step.x++
                    } else if (step.direction === 1) {
                        step.y++
                    } else if (step.direction === 2) {
                        step.x--
                    } else if (step.direction === 3) {
                        step.y--
                    }
                } else if (move === "R") {
                    step.direction = (step.direction + 1) % 4
                } else if (move === "C") {
                    step.direction = (step.direction + 3) % 4
                }
                path.push(step)
            }
            this.agents.push(path)
        }
    }

    // Count the number of tasks reached at each timestep
    count_finished() {
        this.tasks_finished = new Array(this.solution.makespan + 1).fill(0) // We can finish a task at the last timestep
        for (let agent_events of this.solution.events) {
            for (let event of agent_events) {
                if (event[2] === "finished") {
                    const t = event[1]
                    this.tasks_finished[t]++
                }
            }
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

    // Get the task of an agent at a given timestep
    task(agent, time) {
        let cur_task = [-1, -1, -1]
        for (let task of this.solution.events[agent]) {
            if (task[2] === "assigned") {
                if (task[1] <= time && task[1] > cur_task[1]) {
                    cur_task = task
                }
            }
        }
        const coords = this.solution.tasks[cur_task[0]]
        return {
            task: cur_task[0],
            x: coords[2],
            y: coords[1],
        }
    }

    // Direction of an agent at a give time
    direction(agent, time) {
        const p1 = this.agents[agent][time]
        const p2 = this.task(agent, time)
        const v = {x: p2.x - p1.x, y: p2.y - p1.y}
        /*const norm = Math.sqrt((v.x)**2 + (v.y)**2)
        return { x: v.x / norm, y: v.y / norm }*/
        const radians = Math.atan2(v.y, v.x)
        return radians * (180 / Math.PI)
    }
}


class Application {
    constructor() {
        const height = 0.8 * window.innerHeight
        this.svg = SVG().addTo('#app').size(height, height)
        this.svg.group().attr({id: "pixels"})
        this.svg.group().attr({id: "tasks"})
        this.svg.group().attr({id: "agents"})
        this.svg.panZoom({ zoomFactor: 1.2 })
        this.instance = []
        this.bound = 11000
        this.set_instance('random-32-32-20')
        this.update_solution(JSON.stringify(data['solutions'][0]))
    }

    draw_map() {
        const pixels = this.svg.findOne('g#pixels')
        pixels.clear()
        let pattern = this.svg.pattern(10, 10, function(add) {
            add.circle(2).move(4, 4)
        })
        pixels.rect(10*this.instance.map['width'], 10*this.instance.map['height']).attr("fill", pattern)
        pixels.path(this.instance.map['path']).fill('black')
/*        let i = 0
        for (let y = 0; y < this.instance.map['height']; y++) {
            for (let x = 0; x < this.instance.map['width']; x++) {
                if (this.instance.map['bitmap'].charAt(i) === '@' || this.instance.map['bitmap'].charAt(i) === 'T') {
                    pixels.rect(10, 10).move(10*x, 10*y).stroke({ color: 'black', width: 0.1 })
                }
                i++
            }
        }*/
        this.reset_viewer()
    }

    draw_agents() {
        // Draw tasks
        const tasks = this.svg.findOne('g#tasks')
        tasks.clear()
        if (this.instance.solution.teamSize < this.bound) {
            if ('tasks' in this.instance.solution) {
                for (let a = 0; a < this.instance.solution.teamSize; a++) {
                    const t = this.instance.task(a, 0)
                    const circle = tasks.circle(10, {id: `t${a}`}).fill('lightgreen').center(10*t.x + 5, 10*t.y + 5)
                    circle.element('title').words(`t${t.task} of a${a}`)
                }
            }
        }
        // Draw robots
        const agents = this.svg.findOne('g#agents')
        agents.clear()
        if ('start' in this.instance.solution) {
            for (let i = 0; i < this.instance.solution.teamSize; i++) {
                let agent = agents.polygon('5,0 0,-5 -2.5,-2.5 -2.5,2.5 0,5').fill('blue')
                agent.attr({id: `a${i}`, class: 'agent'})
                agent.element('title').words(`a${i}`)
                const position = this.instance.agents[i][0]
                agent.transform({
                    translate: [10*position.x + 5, 10*position.y + 5],
                    origin: [0, 0],
                    rotate: 0
                })
                agent.on('click', () => {
                    console.log('clicked', i)
                    document.querySelector(`#li-a${i}`).scrollIntoView({ behavior: 'smooth' })
                })
                agents.line(0, 0, 5, 0).attr({id: `arrow${i}`}).stroke({ color: '#f06', width: 1, linecap: 'round' }).transform({
                    translate: [10*position.x + 5, 10*position.y + 5],
                    origin: [0, 0],
                    rotate: this.instance.direction(i, 0)
                })
            }
        }
    }

    set_instance(map_name) {
        // const input = document.querySelector('textarea').value
        this.instance = new Instance(map_name, "")
        // this.update_solution()
        this.draw_map()
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

    update_solution(input) {
        this.instance.set_solution(input)
        document.querySelector('input#timestep').setAttribute('max', this.instance.solution.makespan)
        // Make the list of agents
        let li_items = []
        for (let a = 0; a < this.instance.solution.teamSize; a++) {
            li_items.push(`<li id="li-a${a}">Agent a${a}:&nbsp;<span class="coords"></span></li>`)
        }
        document.querySelector('ul#tasks').innerHTML = li_items.join("")
        console.log("Agents drawn")
        // Add tasks
        for (let a = 0; a < this.instance.solution.teamSize; a++) {
            const li = document.querySelector(`li#li-a${a}`)
            for (let task of this.instance.solution.events[a]) {
                if (task[2] === "assigned") {
                    const x = this.instance.solution.tasks[task[0]][2]
                    const y = this.instance.solution.tasks[task[0]][1]
                    const location = this.instance.coords_to_location(x, y)
                    li.innerHTML += `&nbsp;<span id="span-t${task[0]}" class="badge text-bg-primary" title="(${x}, ${y}; ${location})">t${task[0]}</span>`
                }
            }
        }
        console.log("Tasks drawn")
        // Log
        document.querySelector('#log').innerHTML = ""
        document.querySelector('#log').innerHTML += `${this.instance.solution.teamSize} agents\n`
        const ratio = this.instance.solution.numTaskFinished / this.instance.solution.sumOfCost
        document.querySelector('#log').innerHTML += `Delivery ratio: ${ratio.toFixed(6)} (${this.instance.solution.numTaskFinished})\n`
        const idle = this.instance.solution.plannerPaths[0].split(',').reduce((total, x) => (x==='T' ? total+1 : total), 0)
        document.querySelector('#log').innerHTML += `Idle steps: ${idle}/${this.instance.solution.makespan}\n`
        document.querySelector('#log').innerHTML += `Finished tasks: ${this.finished_description()}\n`

        if (this.instance.solution.errors.length > 0) {
            for (const error of this.instance.solution.errors) {
                if (error[3] !== "incorrect vector size") {
                    document.querySelector('h1').style.color = "red"
                    document.querySelector('#log').innerHTML += `Error between a${error[0]} and a${error[1]} at t=${error[2]}: ${error[3]}\n`
                }
            }
        } else {
            document.querySelector('h1').style.color = "black"
        }
        /*if ('plannerTimes' in this.instance.solution) {
            for (let t = 0; t < this.instance.solution.plannerTimes.length; t++) {
                const time = this.instance.solution.plannerTimes[t]
                if (time >= 1) {
                    document.querySelector('#log').innerHTML += `Step at t=${t}: ${time.toFixed(4)}\n`
                }
            }
        }*/
        // Connect robots to ul
        for (let a = 0; a < this.instance.solution.teamSize; a++) {
            document.querySelector(`#li-a${a}`).addEventListener('mouseover', e => {
                this.svg.findOne(`polygon#a${a}`).stroke({ color: 'purple', opacity: 0.7, width: 30 })
                if (this.instance.solution.teamSize < this.bound)
                    this.svg.findOne(`#t${a}`).radius(25).fill({color: 'orange', opacity: 0.7})
            })
            document.querySelector(`#li-a${a}`).addEventListener('mouseout', e => {
                this.svg.findOne(`polygon#a${a}`).stroke({ color: 'red', opacity: 0, width: 0 })
                if (this.instance.solution.teamSize < this.bound)
                    this.svg.findOne(`#t${a}`).radius(5).fill({ color: 'lightgreen', opacity: 1 })
            })
        }
        this.draw_agents()
        this.set_time(0)
    }

    set_time(t) {
        document.querySelector('#timestep').value = t
        for (let a = 0; a < this.instance.solution.teamSize; a++) {
            // Move agent in SVG
            const agent = this.svg.findOne(`#a${a}`)
            const position = this.instance.agents[a][t]
            agent.animate({ when: "now" }).transform({
                translate: [10*position.x + 5, 10*position.y + 5],
                origin: [0, 0],
                rotate: 90*position.direction
            })
            // Move arrow
            const arrow = this.svg.findOne(`#arrow${a}`)
            const angle = this.instance.direction(a, t)
            arrow.stroke(`hsl(${angle} 50% 50%)`).animate({ when: "now" }).transform({
                translate: [10*position.x + 5, 10*position.y + 5],
                origin: [0, 0],
                rotate: angle
            })
            // Update agent in list
            document.querySelector(`#li-a${a} .coords`).textContent = `(${position.x}, ${position.y})`
            // Update task in list
            for (let span of document.querySelectorAll(`#li-a${a} span.badge`)) {
                span.setAttribute('class', 'badge text-bg-secondary')
            }
            let task = this.instance.task(a, t)
            document.querySelector(`#span-t${task.task}`).setAttribute('class', 'badge text-bg-primary')
            // Update task in SVG
            if (this.instance.solution.teamSize < this.bound) {
                const circle = this.svg.findOne(`#t${a}`)
                circle.animate({ when: "now" }).center(10*task.x + 5, 10*task.y + 5)
                circle.findOne('title').words(`t${task.task} of a${a}`)
            }
        }
        console.log(`Time set to ${t}`)
    }

    finished_description() {
        const n = 10
        const makespan = this.instance.solution.makespan + 1
        let description = new Array(n).fill(0)
        for (let i = 0; i < this.instance.tasks_finished.length; i++) {
            let j = Math.floor(i * n / makespan)
            description[j] += this.instance.tasks_finished[i]
        }
        return description.join(", ")
    }

    next_move() {
        const cur_t = Number(document.querySelector('#timestep').value)
        for (let t = cur_t; t < this.instance.solution.makespan; t++) {
            if (!this.instance.wait[t]) {
                this.set_time(t+1)
                return
            }
        }
    }

    previous_move() {
        const cur_t = Number(document.querySelector('#timestep').value)
        for (let t = cur_t-2; t >= 0; t--) {
            if (!this.instance.wait[t]) {
                this.set_time(t+1)
                return
            }
        }
        this.set_time(0)
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