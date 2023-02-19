package com.example.mdp_group_33.ui.main;

import androidx.arch.core.util.Function;
import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;
import androidx.lifecycle.Transformations;
import androidx.lifecycle.ViewModel;

public class PageViewModel extends ViewModel {

    private final MutableLiveData<Integer> myIndex = new MutableLiveData<>();
    private final LiveData<String> myText = Transformations.map(myIndex, new Function<Integer, String>() {
        @Override
        public String apply(Integer input) {
            return "Hello world from section: " + input;
        }
    });

    public void setIndex(int index) {
        myIndex.setValue(index);
    }

    public LiveData<String> getText() {
        return myText;
    }
}