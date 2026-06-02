use legion::{world::SubWorld, *};

use crate::{components::newton_body::NewtonBody, Dt};

#[system]
#[write_component(NewtonBody)]
pub fn newton_body(world: &mut SubWorld, #[resource] dt: &Dt) {
    let mut query = <&mut NewtonBody>::query();
    for body in query.iter_mut(world) {
        body.pos += body.vel * dt.0;
        body.update_a(dt);
    }
}
