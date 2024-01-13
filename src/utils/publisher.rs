use std::{collections::HashSet, hash::Hash};

pub trait Publisher<E> {
    fn notify(&mut self, event: E);
}

pub struct HashsetLedger<E> {
    pub events: HashSet<E>,
}

impl<E: Hash + PartialEq + Eq> Publisher<E> for HashsetLedger<E> {
    fn notify(&mut self, event: E) {
        self.events.insert(event);
    }
}

impl<E> HashsetLedger<E> {
    pub fn new() -> Self {
        Self {
            events: HashSet::new(),
        }
    }
}

impl<E> Default for HashsetLedger<E> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct Ledger<E> {
    pub events: Vec<E>,
}

impl<E> Publisher<E> for Ledger<E> {
    fn notify(&mut self, event: E) {
        self.events.push(event);
    }
}

impl<E> Ledger<E> {
    pub fn new() -> Self {
        Self { events: vec![] }
    }
}

impl<E> Default for Ledger<E> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct CallbackPublisher<E> {
    subscribers: Vec<Box<dyn Fn(&E) + Sync + Send>>,
}

impl<E> CallbackPublisher<E> {
    pub fn new() -> Self {
        Self {
            subscribers: vec![],
        }
    }

    pub fn attach<F: Fn(&E) + Sync + Send + 'static>(&mut self, f: F) {
        self.subscribers.push(Box::new(f));
    }
}

impl<E> Publisher<E> for CallbackPublisher<E> {
    fn notify(&mut self, event: E) {
        self.subscribers.iter().for_each(|f| f(&event));
    }
}

impl<E> Default for CallbackPublisher<E> {
    fn default() -> Self {
        Self::new()
    }
}
