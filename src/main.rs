use plotters::prelude::*;
use rand::Rng;

#[derive(Debug, Clone, Copy)]
pub struct Target {
    x: f64,
    y: f64,
    vx: f64,
    vy: f64,
}

impl Target {
    fn new(x: f64, y: f64, vx: f64, vy: f64) -> Self {
        Target { x, y, vx, vy }
    }

    fn update(&mut self) {
        // Update position based on velocity
        self.x += self.vx;
        self.y += self.vy;
    }

    fn distance_to(&self, other: &Target) -> f64 {
        // Calculate distance to another projectile
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

pub type Interceptor = Target;

// Calculate steering direction towards target (unit vector)
fn calculate_steering_direction(from: &Interceptor, to: &Target) -> (f64, f64) {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let distance = (dx * dx + dy * dy).sqrt();
    
    (dx / distance, dy / distance)

}

// Calculate angle between two velocity vectors in degrees
fn calculate_angle_between_vectors(vx1: f64, vy1: f64, vx2: f64, vy2: f64) -> f64 {
    let dot_product = vx1 * vx2 + vy1 * vy2;
    let magnitude1 = (vx1 * vx1 + vy1 * vy1).sqrt();
    let magnitude2 = (vx2 * vx2 + vy2 * vy2).sqrt();
    
    if magnitude1 > 0.0 && magnitude2 > 0.0 {
        let cos_angle = dot_product / (magnitude1 * magnitude2);
        let angle_rad = cos_angle.acos();
        angle_rad.to_degrees()
    } else {
        0.0
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize projectiles
    let mut target = Target::new(0.0, 30.0, 2.0, 0.0); // Red/Target: 30m height, horizontal
    let mut interceptor = Interceptor::new(0.0, 0.0, 0.0, 0.0); // Green/Interceptor: at ground level
    let interceptor_speed = 2.5; // Speed of interceptor projectile
    let mut rng = rand::thread_rng();

    let mut target_positions = vec![];
    let mut interceptor_positions = vec![];
    let mut collision_point: Option<(f64, f64)> = None;
    let mut collision_angle: Option<f64> = None;

    let collision_threshold = 1.0; // Stop at < 1m distance
    
    let target_initial_height = 30.0; // Initial/target height for correction
    let correction_weight = 0.6; // Weight of correction (0.0 = pure random, 1.0 = pure correction)
    let p_gain = 0.2; // P-Regler Verstärkung (Proportional gain)

    // Simulation for 1000 time steps
    for _step in 0..1000 {
        // Collision detection before update: check if we're already close
        let distance = interceptor.distance_to(&target);
        
        if distance < collision_threshold {
            collision_point = Some((target.x, target.y));
            
            // Calculate angle between velocity vectors
            let angle = calculate_angle_between_vectors(target.vx, target.vy, interceptor.vx, interceptor.vy);
            collision_angle = Some(angle);
            
            break;
        }
        
        // Add random deviation to target's velocity between -5° and +5°
        let random_angle_deg: f64 = rng.gen_range(-5.0..5.0);
        
        // P-Regler: Correction angle proportional to height error
        let height_error = target.y - target_initial_height;
        let correction_angle_deg = -height_error * p_gain; // Negative because we want to correct upward when below target
        
        // Blend random angle and correction angle based on weight
        let blended_angle_deg = (random_angle_deg * (1.0 - correction_weight)) 
                                + (correction_angle_deg * correction_weight);
        
        let random_angle_rad = blended_angle_deg.to_radians();
        
        // Rotate the target's velocity vector by the random angle
        let cos_angle = random_angle_rad.cos();
        let sin_angle = random_angle_rad.sin();
        let rotated_vx = target.vx * cos_angle - target.vy * sin_angle;
        let rotated_vy = target.vx * sin_angle + target.vy * cos_angle;
        
        target.vx = rotated_vx;
        target.vy = rotated_vy;
        
        // Interceptor steers directly towards target
        let (mut dir_x, mut dir_y) = calculate_steering_direction(&interceptor, &target);
        
        // Normalize direction vector
        let dir_magnitude = (dir_x * dir_x + dir_y * dir_y).sqrt();
        if dir_magnitude > 0.0 {
            dir_x /= dir_magnitude;
            dir_y /= dir_magnitude;
        }
        
        interceptor.vx = dir_x * interceptor_speed;
        interceptor.vy = dir_y * interceptor_speed;

        // Update positions
        target.update();
        interceptor.update();

        // Store positions (both X and Y coordinates)
        target_positions.push((target.x, target.y));
        interceptor_positions.push((interceptor.x, interceptor.y));
        
    }

    // Print collision results after simulation ends
    if collision_point.is_some() {
        if let Some((step_x, _)) = collision_point {
            println!("✅ Collision occurred at step {}", step_x as usize);
        }
        if let Some(angle) = collision_angle {
            if angle > 5.0 {
                println!("✅ Angle between velocities is: {:.2}° (greater than 5°)", angle);
            } else {
                println!("❌ Angle between velocities is: {:.2}° (less than 5°)", angle);
            }
        }
    } else {
        println!("❌ No collision occurred within 1000 time steps");
    }

    // Visualization
    visualize_simulation(&target_positions, &interceptor_positions)?;

    Ok(())
}



fn visualize_simulation(
    target_positions: &[(f64, f64)],
    interceptor_positions: &[(f64, f64)],
) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new("collision_simulation.png", (1400, 900)).into_drawing_area();
    root.fill(&WHITE)?;

    // Calculate dynamic boundaries based on data
    let max_x = target_positions
        .iter()
        .chain(interceptor_positions.iter())
        .map(|(x, _)| *x)
        .fold(0.0, f64::max)
        .max(10.0) * 1.1; // Add 10% padding

    let max_y = target_positions
        .iter()
        .chain(interceptor_positions.iter())
        .map(|(_, y)| *y)
        .fold(0.0, f64::max)
        .max(10.0) * 1.1; // Add 10% padding

    let mut chart = ChartBuilder::on(&root)
        .caption("Target vs Interceptor Simulation (Stop at <1m distance)", ("sans-serif", 30))
        .margin(15)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(
            0f64..max_x,
            0f64..max_y,
        )?;

    // Draw target line
    chart
        .draw_series(LineSeries::new(
            target_positions.iter().copied(),
            ShapeStyle::from(&RED).stroke_width(2),
        ))?
        .label("Target (random evasion)");

    // Draw interceptor line
    chart
        .draw_series(LineSeries::new(
            interceptor_positions.iter().copied(),
            ShapeStyle::from(&GREEN).stroke_width(2),
        ))?
        .label("Interceptor (pursuing)");

    // Draw points for target
    for pos in target_positions.iter() {
        chart.draw_series(std::iter::once(Circle::new(
            *pos,
            3,
            ShapeStyle::from(&RED).filled(),
        )))?;
    }

    // Draw points for interceptor
    for pos in interceptor_positions.iter() {
        chart.draw_series(std::iter::once(Circle::new(
            *pos,
            3,
            ShapeStyle::from(&GREEN).filled(),
        )))?;
    }

    // Draw blue circle at the last position of interceptor
    if let Some(&last_interceptor_pos) = interceptor_positions.last() {
        let (collision_x, collision_y) = last_interceptor_pos;
        
        chart.draw_series(std::iter::once(Circle::new(
            (collision_x, collision_y),
            25,
            ShapeStyle::from(&BLUE).stroke_width(3),
        )))?;
    }


    // Configure axes
    chart
        .configure_mesh()
        .x_label_style(("sans-serif", 15))
        .y_label_style(("sans-serif", 15))
        .y_desc("Height (m)")
        .x_desc("Distance (m)")
        .draw()?;

    root.present()?;
    println!("✅ Graph saved as 'collision_simulation.png'");

    Ok(())
}
