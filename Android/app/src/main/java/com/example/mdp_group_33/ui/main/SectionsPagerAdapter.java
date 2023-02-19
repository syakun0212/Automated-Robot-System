package com.example.mdp_group_33.ui.main;

import android.content.Context;

import androidx.annotation.Nullable;
import androidx.annotation.StringRes;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;
import androidx.fragment.app.FragmentPagerAdapter;

import com.example.mdp_group_33.R;

public class SectionsPagerAdapter extends FragmentPagerAdapter {

    @StringRes
    private static final int[] TITLES = new int[]{R.string.tab_text_2, R.string.tab_text_1};
    private final Context myContext;

    public SectionsPagerAdapter(Context context, FragmentManager fm) {
        super(fm);
        myContext = context;
    }

    @Override
    public Fragment getItem(int position) {
        switch(position) {
            case 1:
                return MapControlFragment.newInstance(position + 1);
            default:
                return CommsFragment.newInstance(position + 1);
        }
    }

    @Nullable
    @Override
    public CharSequence getPageTitle(int position) {
        return myContext.getResources().getString(TITLES[position]);
    }

    @Override
    public int getCount() {
        return 2;
    }
}