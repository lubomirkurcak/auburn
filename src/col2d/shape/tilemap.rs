use std::io::{BufReader, BufWriter, Write};

use lk_math::arraynd::Array2d;
use lk_math::modular::ModularDecompose;
use lk_math::vector::V2i32;
use round_to::*;

use serde::{Deserialize, Serialize};
use serde_with::serde_as;

use super::{
    Ball, Box2d, CollidesRel2d, PenetratesRel2d, Point, SymmetricBoundingBox2d, Transformation2d,
    Vec2,
};

use crate::utils::publisher::{Ledger, Publisher};
use crate::utils::rect2::Rect2i32;

const CHUNK_SIZE: V2i32 = V2i32::from_xy(16, 16);

#[derive(Debug)]
pub enum TilemapEvent {
    ChunkCreated(V2i32),
    ChunkRemoved(V2i32),
    ChunkChanged(V2i32),
}

#[serde_as]
#[derive(Default, Serialize, Deserialize)]
pub struct Tilemap {
    // NOTE(lubo): Have to use `std::collections::HashMap` because `serde_as` is not implemented for
    #[serde_as(as = "Vec<(_, _)>")]
    chunks: std::collections::HashMap<V2i32, Chunk>,
    #[serde(skip)]
    // pub events: impl Publisher<TilemapEvent>,
    pub events: Ledger<TilemapEvent>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Chunk {
    pub array2d: Array2d<u8>,
}

impl Default for Chunk {
    fn default() -> Self {
        Self {
            array2d: Array2d::with_dimensions(CHUNK_SIZE.x() as usize, CHUNK_SIZE.y() as usize, 0),
        }
    }
}

impl Tilemap {
    pub fn world_to_tile_pos(&self, world_pos: &Vec2) -> V2i32 {
        V2i32::from_xy(world_pos.x.round_to_i32(), world_pos.y.round_to_i32())
        // V2i32::from_xy(world_pos.x.floor_to_i32(), world_pos.y.floor_to_i32())
    }
    pub fn tile_to_world_pos(&self, tile_pos: &V2i32) -> Vec2 {
        Vec2::new(tile_pos.x() as f32, tile_pos.y() as f32)
    }
    pub fn quantize(&self, t: &Box2d, world_pos: &Vec2) -> Rect2i32 {
        Rect2i32 {
            min: self.world_to_tile_pos(&(*world_pos - t.halfsize)),
            max: self.world_to_tile_pos(&(*world_pos + t.halfsize)),
        }
    }

    pub fn tile_to_chunk_and_local_pos(&self, tile_pos: &V2i32) -> (V2i32, V2i32) {
        tile_pos.modular_decompose(CHUNK_SIZE)
    }

    pub fn tile_to_chunk_pos(&self, tile_pos: &V2i32) -> V2i32 {
        let (chunk_pos, local) = tile_pos.modular_decompose(CHUNK_SIZE);
        chunk_pos
    }

    pub fn get_chunk(&self, chunk_pos: &V2i32) -> Option<&'_ Chunk> {
        self.chunks.get(&chunk_pos)
    }

    pub fn get_tile(&self, pos: V2i32) -> u8 {
        let (chunk_pos, local) = pos.modular_decompose(CHUNK_SIZE);
        if let Some(chunk) = self.chunks.get(&chunk_pos) {
            *chunk.array2d.get(local).unwrap_or(&0)
        } else {
            0
        }
    }

    pub fn set_tile(&mut self, pos: V2i32, tile: u8) {
        let (chunk_pos, local) = pos.modular_decompose(CHUNK_SIZE);
        // let chunk = self.chunks.entry(chunk_pos).or_default();
        let mut created = false;
        let chunk = self.chunks.entry(chunk_pos).or_insert_with(|| {
            created = true;
            self.events.notify(TilemapEvent::ChunkCreated(chunk_pos));
            Chunk::default()
        });
        if let Some(current_tile) = chunk.array2d.get(local) {
            if *current_tile != tile {
                if !created {
                    self.events.notify(TilemapEvent::ChunkChanged(chunk_pos));
                }
                chunk.array2d.set(local, tile);
            }
        }
    }

    pub fn remove_tile(&mut self, pos: V2i32) {
        let (chunk_pos, local) = pos.modular_decompose(CHUNK_SIZE);
        let chunk = self.chunks.entry(chunk_pos).or_default();
        chunk.array2d.set(local, 0);
        if chunk.array2d.data.iter().all(|x| x == &0) {
            self.events.notify(TilemapEvent::ChunkRemoved(chunk_pos));
            self.chunks.remove(&chunk_pos);
        }
    }

    fn for_tiles_in_rect<F: FnMut(V2i32, u8)>(
        &self,
        rect: Rect2i32,
        dir_x: f32,
        dir_y: f32,
        mut f: F,
    ) {
        let flip_x = dir_x < 0.0;
        let flip_y = dir_y < 0.0;
        if flip_y {
            if flip_x {
                for tile_pos in rect.iterate_south_west() {
                    f(tile_pos, self.get_tile(tile_pos));
                }
            } else {
                for tile_pos in rect.iterate_south_east() {
                    f(tile_pos, self.get_tile(tile_pos));
                }
            }
        } else if flip_x {
            for tile_pos in rect.iterate_north_west() {
                f(tile_pos, self.get_tile(tile_pos));
            }
        } else {
            for tile_pos in rect.iterate_north_east() {
                f(tile_pos, self.get_tile(tile_pos));
            }
        }
    }

    pub fn simple_resolve_ball(&self, col: &Ball, mut pos: Vec2, dir: &Vec2) -> Vec2 {
        let bbox = col.symmetric_bounding_box();
        let rect = self.quantize(&bbox, &pos);
        self.for_tiles_in_rect(rect, dir.x, dir.y, |p, t| {
            if t > 0 {
                let b = Box2d::with_halfdims(0.5, 0.5);
                let delta = self.tile_to_world_pos(&p) - pos;
                if let Some(error) = col.penetrates_rel(&b, &delta) {
                    pos += error;
                }
            }
        });
        pos
    }

    pub fn simple_resolve_box(&self, col: &Box2d, mut pos: Vec2, dir: &Vec2) -> Vec2 {
        let rect = self.quantize(&col, &pos);
        self.for_tiles_in_rect(rect, dir.x, dir.y, |p, t| {
            if t > 0 {
                let b = Box2d::with_halfdims(0.5, 0.5);
                let delta = self.tile_to_world_pos(&p) - pos;
                if let Some(error) = col.penetrates_rel(&b, &delta) {
                    pos += error;
                }
            }
        });
        pos
    }

    // pub fn resolve(&self, character: &mut crate::character::Character) {
    //     let rect = self.quantize(&character.col, &character.pos);
    //     self.for_tiles_in_rect(rect, character.vel.x, character.vel.y, |p, t| {
    //         if t > 0 {
    //             let b = Box2d::with_halfdims(0.5, 0.5);
    //             let delta = self.tile_to_world_pos(&p) - character.pos;
    //             if let Some(error) = character.col.penetrates(&b, &delta) {
    //                 character.pos -= error;
    //             }
    //         }
    //     });
    // }
}

impl CollidesRel2d<Point> for Tilemap {
    fn collides_rel(&self, _t: &Point, rel: &impl Transformation2d) -> bool {
        let delta = rel.apply_origin();
        let tile_pos = self.world_to_tile_pos(&delta);
        let tile = self.get_tile(tile_pos);
        tile != 0
    }
}

impl PenetratesRel2d<Point> for Tilemap {
    fn penetrates_rel(&self, _t: &Point, rel: &impl Transformation2d) -> Option<Vec2> {
        let delta = rel.apply_origin();
        let tile_pos = self.world_to_tile_pos(&delta);
        let tile = self.get_tile(tile_pos);
        let up = self.get_tile(tile_pos + V2i32::Y);
        todo!()
    }
}

impl Tilemap {
    pub fn save(&self, filename: &'static str) -> std::io::Result<()> {
        let file = std::fs::File::create(filename)?;
        let mut writer = BufWriter::new(file);
        let serialized = serde_json::to_writer(&mut writer, self)?;
        writer.flush()?;
        Ok(())
    }

    pub fn load(filename: &'static str) -> std::io::Result<Self> {
        let file = std::fs::File::open(filename)?;
        let mut reader = BufReader::new(file);
        Ok(serde_json::from_reader(&mut reader)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lk_math::vector::V2;

    #[test]
    fn serialize_v2() {
        let v = V2::from_xy(3, 2);
        let serialized = serde_json::to_string(&v).unwrap();
        println!("{serialized}");
        let deserialized: V2<i32> = serde_json::from_str(&serialized).unwrap();
        assert_eq!(v, deserialized);
    }

    #[test]
    fn serialize_chunk() {
        let chunk = Chunk::default();
        let serialized = serde_json::to_string(&chunk).unwrap();
        println!("{serialized}");
        let deserialized: Chunk = serde_json::from_str(&serialized).unwrap();
        assert_eq!(chunk.array2d.dims, deserialized.array2d.dims);
        assert_eq!(chunk.array2d.dim_strides, deserialized.array2d.dim_strides);
        assert_eq!(chunk.array2d.data, deserialized.array2d.data);
    }

    #[test]
    fn serialize_tilemap() {
        let mut tilemap = Tilemap::default();
        tilemap.set_tile(lk_math::vector::V2::from_xy(3, 2), 1);
        let serialized = serde_json::to_string(&tilemap).unwrap();
        println!("{serialized}");
        let deserialized: Tilemap = serde_json::from_str(&serialized).unwrap();
        assert_eq!(tilemap.chunks.len(), deserialized.chunks.len());
    }
}
